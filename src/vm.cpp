#include <string>
#include "robot_sim/vm.hpp"
#include "checksum.hpp"
#include <algorithm>
#include <bit>
#include <string_view>
#include <utility>

namespace robot_sim {
namespace {
void require(bool condition, const char* message) {
    if (!condition) throw LoadError(message);
}

struct Reader {
    std::span<const uint8_t> bytes;
    uint8_t byte() { return take(1)[0]; }
    std::span<const uint8_t> take(size_t count) {
        require(count <= bytes.size(), "truncated WASM");
        const auto part = bytes.first(count);
        bytes = bytes.subspan(count);
        return part;
    }
    uint32_t leb(bool sign = false) {
        uint32_t result = 0;
        for (unsigned shift = 0; shift <= 28; shift += 7) {
            const auto b = byte();
            const unsigned payload = b & 127;
            if (shift == 28)
                require(sign ? payload <= 7 || payload >= 120 : payload <= 15,
                        "LEB integer overflow");
            result |= uint32_t(payload) << shift;
            if (!(b & 128)) {
                if (sign && (b & 64) && shift < 28) result |= UINT32_MAX << (shift + 7);
                return result;
            }
        }
        throw LoadError("unterminated LEB integer");
    }
    std::string_view name() {
        const auto value = take(leb());
        for (size_t i = 0; i < value.size();) {
            const auto lead = value[i++];
            if (lead < 0x80) continue;
            require(lead >= 0xc2 && lead <= 0xf4, "invalid UTF-8 name");
            const unsigned trailing = lead < 0xe0 ? 1 : lead < 0xf0 ? 2 : 3;
            require(trailing <= value.size() - i, "truncated UTF-8 name");
            const auto first = value[i];
            require((lead != 0xe0 || first >= 0xa0) && (lead != 0xed || first <= 0x9f) &&
                    (lead != 0xf0 || first >= 0x90) && (lead != 0xf4 || first <= 0x8f),
                    "invalid UTF-8 scalar in name");
            for (unsigned j = 0; j < trailing; ++j)
                require((value[i++] & 0xc0) == 0x80, "invalid UTF-8 continuation byte");
        }
        return {reinterpret_cast<const char*>(value.data()), value.size()};
    }
    uint32_t count(uint32_t max = 1024) {
        const auto n = leb();
        require(n <= max, "WASM count exceeds profile limit");
        return n;
    }
    uint32_t constant() {
        require(byte() == 0x41, "expected i32.const initializer");
        const auto n = leb(true);
        require(byte() == 0x0b, "bad initializer terminator");
        return n;
    }
};

struct Instruction { uint8_t op; uint32_t a = 0; size_t target = 0; };
struct Signature { uint32_t params; bool result; };
struct Function { uint32_t type, locals = 0; std::vector<Instruction> code; };
struct Global { uint32_t value; bool writable; };
struct Segment { uint32_t offset; std::vector<uint8_t> bytes; };
unsigned width(uint8_t op) {
    switch (op) {
    case 0x28: case 0x36: return 4;
    case 0x2c: case 0x2d: case 0x3a: return 1;
    case 0x2e: case 0x2f: case 0x3b: return 2;
    default: return 0;
    }
}
}

struct Module {
    uint32_t initial = 0, maximum = UINT32_MAX, entry = UINT32_MAX;
    bool memory = false;
    WasmFeatures features;
    std::vector<Signature> types;
    std::vector<Function> functions;
    std::vector<Global> globals;
    std::vector<Segment> data;
    Signature signature(size_t index) const { return types[functions[index].type]; }
};

namespace {
void decode(Reader r, const Module& module, Function& f) {
    const auto sig = module.types[f.type];
    f.locals = sig.params;
    for (auto n = r.count(); n; --n) {
        f.locals += r.count();
        require(f.locals <= 1024 && r.byte() == 0x7f, "unsupported locals");
    }
    struct Label {
        uint8_t op;
        size_t instruction;
        unsigned height;
        bool dead = false, has_else = false;
        std::vector<size_t> branches;
    };
    std::vector<Label> labels{{0, 0, 0}};
    unsigned height = 0;
    auto pop = [&](unsigned n) {
        const auto base = labels.back().height;
        if (n > height - base) {
            require(labels.back().dead, "operand stack underflow");
            height = base;
        } else height -= n;
    };
    auto dead = [&] { height = labels.back().height; labels.back().dead = true; };
    auto finish = [&](unsigned result) {
        pop(result);
        require(height == labels.back().height, "invalid block stack height");
    };
    while (!labels.empty()) {
        require(f.code.size() < 65536, "function too large");
        Instruction i{r.byte()};
        const auto pc = f.code.size();
        switch (i.op) {
        case 0x00: dead(); break;
        case 0x01: break;
        case 0x02: case 0x03: case 0x04:
            require(r.byte() == 0x40 && labels.size() < 256, "unsupported block signature or depth");
            if (i.op == 0x04) pop(1);
            {
                labels.push_back({i.op, pc, height});
            }
            break;
        case 0x05: {
            auto& label = labels.back();
            require(label.op == 0x04 && !label.has_else, "unexpected else");
            finish(0);
            f.code[label.instruction].target = pc + 1;
            label.has_else = true;
            label.dead = false;
            label.branches.push_back(pc);
            break;
        }
        case 0x0b: {
            const auto& label = labels.back();
            finish(label.op == 0 && sig.result ? 1 : 0);
            if (label.op == 0x04 && !label.has_else) f.code[label.instruction].target = pc + 1;
            for (auto branch : label.branches) {
                f.code[branch].target = label.op == 0 ? pc : pc + 1;
            }
            labels.pop_back();
            break;
        }
        case 0x0c: case 0x0d: {
            const auto depth = r.leb();
            require(depth < labels.size(), "invalid branch depth");
            if (i.op == 0x0d) pop(1);
            auto& label = labels[labels.size() - 1 - depth];
            if (label.op == 0 && sig.result) {
                pop(1);
                if (i.op == 0x0d) ++height;
                i.a = 1;
            }
            i.a |= label.height << 1;
            if (label.op == 0x03) i.target = label.instruction + 1;
            else label.branches.push_back(pc);
            if (i.op == 0x0c) dead();
            break;
        }
        case 0x0f: pop(sig.result); dead(); break;
        case 0x10: {
            i.a = r.leb();
            require(i.a < module.functions.size(), "invalid call index");
            const auto callee = module.signature(i.a);
            pop(callee.params);
            height += callee.result;
            break;
        }
        case 0x1a: pop(1); break;
        case 0x1b: pop(3); ++height; break;
        case 0x20: case 0x21: case 0x22:
            i.a = r.leb();
            require(i.a < f.locals, "invalid local index");
            if (i.op == 0x21) pop(1);
            else if (i.op == 0x22) { pop(1); ++height; }
            else ++height;
            break;
        case 0x23: case 0x24:
            i.a = r.leb();
            require(i.a < module.globals.size(), "invalid global index");
            if (i.op == 0x23) ++height;
            else {
                require(module.globals[i.a].writable, "immutable global");
                pop(1);
            }
            break;
        case 0x3f: case 0x40:
            require(module.memory && r.byte() == 0, "invalid memory index");
            if (i.op == 0x40) pop(1);
            ++height;
            break;
        case 0x41: i.a = r.leb(true); ++height; break;
        case 0x45: pop(1); ++height; break;
        case 0xc0: case 0xc1:
            require(module.features.sign_extension, "i32 sign-extension feature disabled");
            pop(1);
            ++height;
            break;
        default:
            if (const auto bytes = width(i.op)) {
                require(module.memory && r.leb() <= unsigned(std::countr_zero(bytes)),
                        "invalid memory instruction");
                i.a = r.leb();
                if (i.op >= 0x36) pop(2);
                else { pop(1); ++height; }
            } else if ((i.op >= 0x46 && i.op <= 0x4f) ||
                       (i.op >= 0x6a && i.op <= 0x78)) {
                pop(2);
                ++height;
            } else throw LoadError("unsupported opcode " + std::to_string(i.op));
        }
        require(height <= 1024, "operand stack exceeds profile limit");
        f.code.push_back(i);
    }
    require(r.bytes.empty(), "trailing bytes in function body");
}
}

std::shared_ptr<const Module> validate(std::span<const uint8_t> bytes, WasmFeatures features) {
    require(bytes.size() <= 4 * 1024 * 1024, "module too large");
    Reader r{bytes};
    const uint8_t header[] = {0, 97, 115, 109, 1, 0, 0, 0};
    require(std::ranges::equal(r.take(8), header), "invalid WASM header or version");
    auto m = std::make_shared<Module>();
    m->features = features;
    uint8_t previous = 0;
    bool code_seen = false;
    while (!r.bytes.empty()) {
        const auto id = r.byte();
        Reader s{r.take(r.leb())};
        if (!id) { s.name(); continue; }
        require(id > previous, "duplicate or out-of-order section");
        previous = id;
        switch (id) {
        case 1:
            for (auto n = s.count(); n; --n) {
                require(s.byte() == 0x60, "unsupported type");
                const auto params = s.count(64);
                for (uint32_t i = 0; i < params; ++i)
                    require(s.byte() == 0x7f, "only i32 parameters supported");
                const bool result = s.count(1) != 0;
                require(!result || s.byte() == 0x7f, "only i32 results supported");
                m->types.push_back({params, result});
            }
            break;
        case 3:
            for (auto n = s.count(); n; --n) {
                const auto type = s.leb();
                require(type < m->types.size(), "invalid function type");
                m->functions.push_back({type});
            }
            break;
        case 5:
            require(s.count(1) == 1, "expected single memory");
            m->memory = true;
            {
                const bool bounded = s.count(1) != 0;
                m->initial = s.count(65536);
                if (bounded) {
                    m->maximum = s.count(65536);
                    require(m->maximum >= m->initial, "invalid memory limits");
                }
            }
            break;
        case 6:
            for (auto n = s.count(); n; --n) {
                require(s.byte() == 0x7f, "only i32 globals supported");
                const bool writable = s.count(1) != 0;
                m->globals.push_back({s.constant(), writable});
            }
            break;
        case 7: {
            std::vector<std::string_view> names;
            for (auto n = s.count(); n; --n) {
                const auto name = s.name();
                require(std::ranges::find(names, name) == names.end(), "duplicate export name");
                names.push_back(name);
                const auto kind = s.byte();
                const auto index = s.leb();
                require((kind == 0 && index < m->functions.size()) ||
                        (kind == 2 && index == 0 && m->memory), "unsupported or invalid export");
                if (name == "robot_main_v1") {
                    require(kind == 0 && m->signature(index).params == 0 &&
                            !m->signature(index).result, "entry must have signature () -> ()");
                    m->entry = index;
                }
            }
            break;
        }
        case 10:
            require(s.count() == m->functions.size(), "code/function count mismatch");
            for (auto& f : m->functions) decode(Reader{s.take(s.leb())}, *m, f);
            code_seen = true;
            break;
        case 11:
            for (auto n = s.count(); n; --n) {
                require(m->memory && s.leb() == 0, "expected active memory-zero data");
                const auto offset = s.constant();
                const auto data = s.take(s.leb());
                require(uint64_t(offset) + data.size() <= uint64_t(m->initial) * Bus::page_size,
                        "data segment exceeds initial memory");
                m->data.push_back({offset, {data.begin(), data.end()}});
            }
            break;
        default: throw LoadError("unsupported section " + std::to_string(id));
        }
        require(s.bytes.empty(), "trailing bytes in section");
    }
    require(code_seen && m->entry != UINT32_MAX, "missing robot_main_v1 entry or code");
    return m;
}

Vm::Vm(std::shared_ptr<const Module> module, uint32_t ram_cap)
    : module_(std::move(module)),
      bus_(module_ ? module_->initial : 0, module_ ? std::min(ram_cap, module_->maximum) : 0) {
    if (!module_) throw std::invalid_argument("missing validated module");
    for (const auto& g : module_->globals) globals_.push_back(g.value);
    for (const auto& segment : module_->data)
        for (size_t i = 0; i < segment.bytes.size(); ++i)
            bus_.store(segment.offset + i, 1, segment.bytes[i]);
    call(module_->entry);
}

void Vm::call(size_t function) {
    if (frames_.size() >= 128) throw std::runtime_error("call stack limit exceeded");
    const auto params = module_->signature(function).params;
    const auto base = stack_.size() - params;
    Frame frame{function, 0, base, std::vector<uint32_t>(module_->functions[function].locals)};
    std::copy(stack_.begin() + base, stack_.end(), frame.locals.begin());
    stack_.resize(base);
    frames_.push_back(std::move(frame));
}

namespace {
uint32_t binary(uint8_t op, uint32_t a, uint32_t b) {
    const auto x = std::bit_cast<int32_t>(a), y = std::bit_cast<int32_t>(b);
    switch (op) {
    case 0x46: return a == b;
    case 0x47: return a != b;
    case 0x48: return x < y;
    case 0x49: return a < b;
    case 0x4a: return x > y;
    case 0x4b: return a > b;
    case 0x4c: return x <= y;
    case 0x4d: return a <= b;
    case 0x4e: return x >= y;
    case 0x4f: return a >= b;
    case 0x6a: return a + b;
    case 0x6b: return a - b;
    case 0x6c: return a * b;
    case 0x6d:
        if (!b || (x == INT32_MIN && y == -1)) break;
        return uint32_t(x / y);
    case 0x6e: if (b) return a / b; break;
    case 0x6f: if (b) return y == -1 ? 0 : uint32_t(x % y); break;
    case 0x70: if (b) return a % b; break;
    case 0x71: return a & b;
    case 0x72: return a | b;
    case 0x73: return a ^ b;
    case 0x74: return a << (b & 31);
    case 0x75: return uint32_t(x >> (b & 31));
    case 0x76: return a >> (b & 31);
    case 0x77: return std::rotl(a, int(b & 31));
    case 0x78: return std::rotr(a, int(b & 31));
    }
    throw std::runtime_error("integer arithmetic trap");
}
}

RunState Vm::step(uint32_t active_units) {
    uint64_t allowance = 0;
    const auto mode = bus_.power();
    if (mode != Power::sleep) {
        const auto halves = uint64_t(active_units) * (mode == Power::low_power ? 1 : 2) + half_cu_;
        allowance = halves / 2;
        half_cu_ = halves % 2;
    }
    const auto state = run(allowance);
    bus_.tick();
    return state;
}

uint64_t Vm::checksum() const {
    Checksum hash;
    hash.word(bus_.checksum());
    hash.word(instructions_);
    hash.word(bool(trap_));
    hash.word(globals_.size());
    for (auto value : globals_) hash.word(value);
    hash.word(stack_.size());
    for (auto value : stack_) hash.word(value);
    hash.word(frames_.size());
    for (const auto& frame : frames_) {
        hash.word(frame.function);
        hash.word(frame.pc);
        hash.word(frame.stack_base);
        hash.word(frame.locals.size());
        for (auto value : frame.locals) hash.word(value);
    }
    return hash.value();
}

std::optional<ExecutionLocation> Vm::location() const {
    if (frames_.empty()) return {};
    const auto& frame = frames_.back();
    return ExecutionLocation{frame.function, frame.pc - (trap_ && frame.pc ? 1 : 0)};
}

RunState Vm::run(uint64_t budget) try {
    if (trap_) std::rethrow_exception(trap_);
    auto pop = [&] { const auto v = stack_.back(); stack_.pop_back(); return v; };
    while (!frames_.empty()) {
        if (bus_.power() == Power::sleep) return RunState::sleeping;
        if (!budget) return RunState::budget;
        auto& frame = frames_.back();
        const auto& code = module_->functions[frame.function].code;
        const auto i = code[frame.pc++];
        --budget;
        ++instructions_;
        switch (i.op) {
        case 0x00: throw std::runtime_error("guest unreachable");
        case 0x01: case 0x02: case 0x03: break;
        case 0x04: if (!pop()) frame.pc = i.target; break;
        case 0x05: frame.pc = i.target; break;
        case 0x0b: if (frame.pc != code.size()) break; [[fallthrough]];
        case 0x0f: {
            const bool result = module_->signature(frame.function).result;
            const auto value = result ? pop() : 0;
            stack_.resize(frame.stack_base);
            frames_.pop_back();
            if (result) stack_.push_back(value);
            break;
        }
        case 0x0c: case 0x0d:
            if ((i.op & 1) && !pop()) break;
            {
                const auto result = (i.a & 1) ? pop() : 0;
                stack_.resize(frame.stack_base + (i.a >> 1));
                if (i.a & 1) stack_.push_back(result);
                frame.pc = i.target;
            }
            break;
        case 0x10: call(i.a); break;
        case 0x1a: pop(); break;
        case 0x1b: {
            const auto condition = pop(), b = pop(), a = pop();
            stack_.push_back(condition ? a : b);
            break;
        }
        case 0x20: stack_.push_back(frame.locals[i.a]); break;
        case 0x21: frame.locals[i.a] = pop(); break;
        case 0x22: frame.locals[i.a] = stack_.back(); break;
        case 0x23: stack_.push_back(globals_[i.a]); break;
        case 0x24: globals_[i.a] = pop(); break;
        case 0x3f: stack_.push_back(bus_.pages()); break;
        case 0x40: stack_.push_back(bus_.grow(pop())); break;
        case 0x41: stack_.push_back(i.a); break;
        case 0x45: stack_.push_back(pop() == 0); break;
        case 0xc0: {
            const auto value = pop() & 0xffu;
            stack_.push_back((value ^ 0x80u) - 0x80u);
            break;
        }
        case 0xc1: {
            const auto value = pop() & 0xffffu;
            stack_.push_back((value ^ 0x8000u) - 0x8000u);
            break;
        }
        default:
            if (const auto bytes = width(i.op)) {
                if (i.op >= 0x36) {
                    const auto value = pop();
                    bus_.store(uint64_t(pop()) + i.a, bytes, value);
                } else {
                    auto value = uint32_t(bus_.load(uint64_t(pop()) + i.a, bytes));
                    if (i.op == 0x2c && (value & 0x80)) value |= 0xffffff00u;
                    if (i.op == 0x2e && (value & 0x8000)) value |= 0xffff0000u;
                    stack_.push_back(value);
                }
            } else {
                const auto b = pop(), a = pop();
                stack_.push_back(binary(i.op, a, b));
            }
        }
    }
    return RunState::finished;
} catch (...) {
    trap_ = std::current_exception();
    throw;
}
}

