#include "cli_scenario.hpp"
#include "scenario_json.hpp"
#include <charconv>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace robot_sim;

namespace {
constexpr std::streamoff max_trace_bytes = 64 * 1024 * 1024;
uint64_t integer(std::string_view value) {
    uint64_t result = 0;
    const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), result);
    if (error != std::errc{} || end != value.data() + value.size())
        throw std::invalid_argument("invalid integer option");
    return result;
}
uint32_t limit(std::string_view value, uint32_t maximum) {
    const auto n = integer(value);
    if (n > maximum) throw std::invalid_argument("option exceeds limit");
    return static_cast<uint32_t>(n);
}
const char* mode(Power power) {
    if (power == Power::sleep) return "sleep";
    if (power == Power::low_power) return "low_power";
    return "active";
}
const char* state(const RobotSnapshot& robot) {
    if (!robot.fault.empty()) return "fault";
    if (robot.state == RunState::finished) return "finished";
    return robot.power == Power::sleep ? "sleeping" : "budget";
}
void json_string(std::ostream& out, std::string_view value) {
    out << '"';
    for (auto ch : value) {
        if (ch == '"' || ch == '\\') out << '\\' << ch;
        else if (static_cast<unsigned char>(ch) < 32) out << '?';
        else out << ch;
    }
    out << '"';
}

struct TraceHeader {
    uint64_t module_hash = 0, seed = 0, active = 0, low = 0;
    uint32_t rate = 0, ticks = 0, pages = 0, dma = 0;
    uint32_t protocol = 1, address = 7, timeout = 8;
    uint32_t clock = 0, data = 1, peripheral_clock = 0, peripheral_data = 1;
};

std::ostream& operator<<(std::ostream& out, const TraceHeader& header) {
    return out << "robot-sim-trace-v1 cu-v1 " << header.module_hash << ' ' << header.seed
               << ' ' << header.active << ' ' << header.low << ' ' << header.rate
               << ' ' << header.ticks << ' ' << header.pages << ' ' << header.dma
               << ' ' << header.protocol << ' ' << header.address << ' ' << header.timeout
               << ' ' << header.clock << ' ' << header.data << ' ' << header.peripheral_clock
               << ' ' << header.peripheral_data << '\n';
}
void read_header(std::istream& stream, TraceHeader& header) {
    std::string format, cost;
    if (!(stream >> format >> cost >> header.module_hash >> header.seed >> header.active
          >> header.low >> header.rate >> header.ticks >> header.pages >> header.dma
          >> header.protocol >> header.address >> header.timeout >> header.clock
          >> header.data >> header.peripheral_clock >> header.peripheral_data) ||
        format != "robot-sim-trace-v1" || cost != "cu-v1")
        throw std::invalid_argument("invalid trace header");
}

void robot_json(const std::filesystem::path& path, TraceHeader& header) {
    for (const auto& [key, value] : scenario_json::read(path)) {
        if (key == "active_cu_per_second") header.active = scenario_json::number(value);
        else if (key == "low_power_cu_per_second") header.low = scenario_json::number(value);
        else {
            const auto word = static_cast<uint32_t>(scenario_json::number(value, UINT32_MAX));
            if (key == "ram_pages") header.pages = word;
            else if (key == "protocol") header.protocol = word;
            else if (key == "target") header.address = word;
            else if (key == "timeout_ticks") header.timeout = word;
            else if (key == "controller_clock") header.clock = word;
            else if (key == "controller_data") header.data = word;
            else if (key == "peripheral_clock") header.peripheral_clock = word;
            else if (key == "peripheral_data") header.peripheral_data = word;
            else if (key == "dma_max_bytes") header.dma = word;
            else throw std::invalid_argument("unknown robot field " + key);
        }
    }
}

void write_tick(std::ostream& out, const WorldInput& input, const StepResult& step) {
    out << step.tick << ' ' << step.checksum << ' ' << input.completions.size();
    if (!input.completions.empty()) {
        const auto& response = input.completions.front();
        out << ' ' << response.request << ' ' << response.status << ' ' << response.response
            << ' ' << response.payload.size();
        for (auto byte : response.payload) out << ' ' << unsigned(byte);
    }
    out << ' ' << (input.irqs.empty() ? 0 : input.irqs.front().mask) << '\n';
}
WorldInput read_tick(std::istream& in, RobotId robot, uint64_t tick, uint64_t& checksum) {
    uint64_t actual = 0;
    unsigned count = 0;
    if (!(in >> actual >> checksum >> count) || actual != tick || count > 1)
        throw std::invalid_argument("invalid trace tick");
    WorldInput input;
    if (count) {
        Completion response{robot};
        unsigned size = 0;
        if (!(in >> response.request >> response.status >> response.response >> size) || size > 65536)
            throw std::invalid_argument("invalid trace response");
        for (unsigned i = 0; i < size; ++i) {
            unsigned byte = 0;
            if (!(in >> byte) || byte > 255) throw std::invalid_argument("invalid trace payload");
            response.payload.push_back(static_cast<uint8_t>(byte));
        }
        input.completions.push_back(std::move(response));
    }
    uint32_t irq = 0;
    if (!(in >> irq)) throw std::invalid_argument("invalid trace IRQ");
    if (irq) input.irqs.push_back({robot, irq});
    return input;
}

WorldInput mock_response(const StepResult& step, uint64_t seed) {
    WorldInput input;
    for (const auto& command : step.commands) {
        const auto& request = command.transfer;
        Completion response{command.robot, command.request, 2, request.command ^ 0xa5u};
        if (request.direction == 1) {
            response.payload.resize(request.dma_size);
            for (size_t i = 0; i < response.payload.size(); ++i)
                response.payload[i] = static_cast<uint8_t>(seed + request.target + i);
        }
        input.completions.push_back(std::move(response));
    }
    return input;
}
}

int run_scenario(std::string_view command, std::span<char*> args,
                 std::shared_ptr<const Module> module, uint64_t module_hash) {
    TraceHeader header{module_hash, 0, 30000, 15000, 30, 1, 16, 0};
    uint32_t workers = 1;
    std::filesystem::path json, trace, replay, scenario;
    std::map<std::string, uint64_t> expectations;
    for (size_t i = 0; i < args.size(); i += 2) {
        if (i + 1 >= args.size()) throw std::invalid_argument("option requires a value");
        const std::string_view key = args[i], value = args[i + 1];
        if (key == "--ticks") header.ticks = limit(value, 1000000);
        else if (key == "--tick-rate") header.rate = limit(value, 1000000000);
        else if (key == "--workers") workers = limit(value, 64);
        else if (key == "--active-cu") header.active = integer(value);
        else if (key == "--low-cu") header.low = integer(value);
        else if (key == "--ram-pages") header.pages = limit(value, 16384);
        else if (key == "--dma-bytes") header.dma = limit(value, 65536);
        else if (key == "--seed") header.seed = integer(value);
        else if (key == "--json") json = value;
        else if (key == "--robot") robot_json(value, header);
        else if (key == "--scenario") scenario = value;
        else if (key == "--trace") trace = value;
        else if (key == "--replay") replay = value;
        else if (key == "--expect-status" || key == "--expect-wakes" ||
                 key == "--expect-commands" || key == "--expect-alarms" ||
                 key == "--expect-trap" || key == "--expect-power")
            expectations[std::string(key)] = integer(value);
        else throw std::invalid_argument("unknown option " + std::string(key));
    }
    std::ifstream playback;
    std::ofstream recording;
    std::map<uint32_t, uint32_t> scripted_irqs;
    if (!scenario.empty()) {
        if (!replay.empty()) throw std::invalid_argument("a replay already contains scenario inputs");
        for (const auto& [key, value] : scenario_json::read(scenario)) {
            if (key == "ticks") header.ticks = static_cast<uint32_t>(scenario_json::number(value, 1000000));
            else if (key == "tick_rate") header.rate = static_cast<uint32_t>(scenario_json::number(value, 1000000000));
            else if (key == "seed") header.seed = scenario_json::number(value);
            else if (key == "workers") workers = static_cast<uint32_t>(scenario_json::number(value, 64));
            else if (key == "irqs") {
                const auto* events = std::get_if<scenario_json::Value::Array>(&value.data);
                if (!events) throw std::invalid_argument("irqs must be an array");
                for (const auto& event : *events) {
                    const auto* fields = std::get_if<scenario_json::Value::Object>(&event.data);
                    if (!fields || fields->size() != 2) throw std::invalid_argument("invalid IRQ event");
                    const auto tick = static_cast<uint32_t>(scenario_json::field(*fields, "tick", 1000000));
                    const auto mask = static_cast<uint32_t>(scenario_json::field(*fields, "mask", UINT32_MAX));
                    if (!tick || !scripted_irqs.emplace(tick, mask).second)
                        throw std::invalid_argument("invalid or duplicate scripted IRQ tick");
                }
            } else if (key == "expect") {
                const auto* fields = std::get_if<scenario_json::Value::Object>(&value.data);
                if (!fields) throw std::invalid_argument("expect must be an object");
                for (const auto& [assertion, expected] : *fields) {
                    if (assertion != "status" && assertion != "wakes" && assertion != "commands" &&
                        assertion != "alarms" && assertion != "power" && assertion != "trap")
                        throw std::invalid_argument("unknown scenario assertion " + assertion);
                    expectations.try_emplace("--expect-" + assertion, scenario_json::number(expected));
                }
            } else throw std::invalid_argument("unknown scenario field " + key);
        }
    }
    if (!replay.empty()) {
        if (!trace.empty()) throw std::invalid_argument("choose trace recording or replay");
        playback.open(replay, std::ios::ate);
        const auto size = playback.tellg();
        if (size < 0 || size > max_trace_bytes) throw std::invalid_argument("cannot open replay or file exceeds 64 MiB");
        playback.seekg(0);
        read_header(playback, header);
        if (header.module_hash != module_hash) throw std::invalid_argument("trace module mismatch");
    }
    if (!header.ticks || header.ticks > 1000000) throw std::invalid_argument("invalid tick count");
    Simulation simulation({header.rate, workers});
    if (header.protocol > 1) throw std::invalid_argument("unsupported protocol");
    RobotConfig config{header.active, header.low, header.pages, {}};
    if (header.protocol) config.controller = ControllerConfig{Protocol::abstract,
        {header.clock, header.data}, {header.peripheral_clock, header.peripheral_data},
        header.address, header.timeout, header.dma};
    const auto robot = simulation.add_robot(config, std::move(module));
    if (!trace.empty()) {
        recording.open(trace);
        if (!recording) throw std::runtime_error("could not open trace");
        recording << header;
    }
    const auto start = std::chrono::steady_clock::now();
    StepResult result{};
    WorldInput input;
    uint64_t commands = 0;
    for (uint64_t tick = 1; tick <= header.ticks; ++tick) {
        uint64_t checksum = 0;
        if (playback.is_open()) input = read_tick(playback, robot, tick, checksum);
        else if (const auto irq = scripted_irqs.find(static_cast<uint32_t>(tick)); irq != scripted_irqs.end())
            input.irqs.push_back({robot, irq->second});
        result = simulation.step(input);
        commands += result.commands.size();
        if (playback.is_open() && result.checksum != checksum) {
            std::cerr << "replay checksum mismatch at tick " << tick << '\n';
            return 5;
        }
        if (recording.is_open()) {
            const auto offset = recording.tellp();
            if (offset < 0 || offset > max_trace_bytes - 512 * 1024)
                throw std::runtime_error("trace exceeded recording limit");
            write_tick(recording, input, result);
        }
        if (!playback.is_open()) input = mock_response(result, header.seed);
    }
    if (recording.is_open()) {
        recording.close();
        if (!recording) throw std::runtime_error("could not finish writing trace");
    }
    if (playback.is_open() && (playback >> std::ws).peek() != EOF)
        throw std::invalid_argument("unexpected trailing trace data");
    const auto host_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    const auto report = simulation.inspect(robot);
    std::ostringstream output;
    output << "{\"version\":1,\"robot\":" << robot << ",\"ticks\":" << result.tick
           << ",\"rtc_ns\":" << report.rtc_ns << ",\"checksum\":\"" << result.checksum
           << "\",\"state\":\"" << state(report) << "\",\"power\":\"" << mode(report.power)
           << "\",\"instructions\":" << report.instructions << ",\"cu\":" << report.instructions
           << ",\"mmio_accesses\":" << report.mmio_accesses << ",\"pending_irq\":" << report.pending_irq
           << ",\"active_ticks\":" << report.active_ticks << ",\"low_power_ticks\":" << report.low_power_ticks
           << ",\"sleep_ticks\":" << report.sleeping_ticks << ",\"wakes\":" << report.wakes
           << ",\"active_ns\":" << report.active_ns << ",\"low_power_ns\":" << report.low_power_ns
           << ",\"sleep_ns\":" << report.sleeping_ns
           << ",\"alarms\":" << report.alarms << ",\"ram_pages\":" << report.ram_pages
           << ",\"peak_ram_bytes\":" << uint64_t(report.ram_pages) * Bus::page_size
           << ",\"commands\":" << commands << ",\"transactions\":" << report.transactions
           << ",\"completed\":" << report.completed << ",\"protocol_errors\":" << report.protocol_errors
           << ",\"dma_bytes\":" << report.dma_bytes << ",\"status\":" << report.status
           << ",\"response\":" << report.response << ",\"fault\":";
    json_string(output, report.fault);
    if (report.location)
        output << ",\"function\":" << report.location->function
               << ",\"instruction\":" << report.location->instruction;
    if (report.memory_fault)
        output << ",\"fault_kind\":" << static_cast<unsigned>(*report.memory_fault)
               << ",\"fault_address\":" << *report.fault_address;
    if (command == "profile") output << ",\"host_ms\":" << host_ms;
    output << "}\n";
    if (!json.empty()) {
        std::ofstream file(json);
        file << output.str();
        file.close();
        if (!file) throw std::runtime_error("could not write JSON report");
    } else std::cout << output.str();

    for (const auto& [key, expected] : expectations) {
        uint64_t actual = UINT64_MAX;
        if (key == "--expect-status") actual = report.status;
        if (key == "--expect-wakes") actual = report.wakes;
        if (key == "--expect-commands") actual = commands;
        if (key == "--expect-alarms") actual = report.alarms;
        if (key == "--expect-power") actual = static_cast<uint32_t>(report.power);
        if (key == "--expect-trap" && report.memory_fault)
            actual = static_cast<uint32_t>(*report.memory_fault);
        if (actual != expected) {
            std::cerr << key << " expected " << expected << ", observed " << actual << '\n';
            return 5;
        }
    }
    return report.fault.empty() || (command == "test" && expectations.contains("--expect-trap")) ? 0 : 4;
}

