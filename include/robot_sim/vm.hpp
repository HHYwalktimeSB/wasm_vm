#pragma once
#include "robot_sim/bus.hpp"
#include <memory>
#include <span>
#include <exception>

namespace robot_sim {
struct Module;
struct LoadError : std::runtime_error { using std::runtime_error::runtime_error; };
struct WasmFeatures { bool sign_extension = false; };
std::shared_ptr<const Module> validate(std::span<const uint8_t> bytes, WasmFeatures features = {});
enum class RunState { finished, budget, sleeping };
struct ExecutionLocation { size_t function, instruction; };

class Vm {
public:
    explicit Vm(std::shared_ptr<const Module> module, uint32_t ram_cap = 16);
    RunState run(uint64_t budget);
    RunState step(uint32_t active_units);
    Bus& bus() { return bus_; }
    const Bus& bus() const { return bus_; }
    uint64_t instructions() const { return instructions_; }
    uint64_t checksum() const;
    std::optional<ExecutionLocation> location() const;
private:
    struct Frame { size_t function, pc, stack_base; std::vector<uint32_t> locals; };
    void call(size_t function);
    std::shared_ptr<const Module> module_;
    Bus bus_;
    std::vector<uint32_t> stack_, globals_;
    std::vector<Frame> frames_;
    uint64_t instructions_ = 0;
    unsigned half_cu_ = 0;
    std::exception_ptr trap_;
};
}

