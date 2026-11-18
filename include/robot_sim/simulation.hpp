#pragma once
#include "robot_sim/vm.hpp"
#include <map>
#include <set>
#include <string>

namespace robot_sim {
using RobotId = uint64_t;

struct SimulatorConfig {
    uint32_t ticks_per_second = 30;
    uint32_t workers = 1;
};

struct RobotConfig {
    uint64_t active_cu_per_second = 100000;
    uint64_t low_power_cu_per_second = 50000;
    uint32_t ram_pages = 16;
    std::optional<ControllerConfig> controller;
};

struct Completion {
    RobotId robot;
    uint64_t request;
    uint32_t status = 2, response = 0;
    std::vector<uint8_t> payload;
};
struct ExternalIrq { RobotId robot; uint32_t mask; };
struct WorldInput {
    std::vector<Completion> completions;
    std::vector<ExternalIrq> irqs;
};
struct DeviceCommand {
    RobotId robot;
    uint64_t request;
    Transfer transfer;
};
struct RobotSnapshot {
    RunState state = RunState::budget;
    Power power = Power::active;
    uint64_t instructions = 0, mmio_accesses = 0, alarms = 0;
    uint64_t active_ticks = 0, low_power_ticks = 0, sleeping_ticks = 0, wakes = 0;
    uint64_t active_ns = 0, low_power_ns = 0, sleeping_ns = 0;
    uint64_t transactions = 0, completed = 0, protocol_errors = 0, dma_bytes = 0;
    uint32_t ram_pages = 0, pending_irq = 0;
    uint32_t status = 0, response = 0;
    uint64_t rtc_ns = 0;
    std::optional<ExecutionLocation> location;
    std::string fault;
    std::optional<FaultKind> memory_fault;
    std::optional<uint64_t> fault_address;
};
struct StepResult {
    uint64_t tick, time_ns, checksum;
    std::vector<DeviceCommand> commands;
};

class Simulation {
public:
    explicit Simulation(SimulatorConfig config);
    ~Simulation();
    RobotId add_robot(const RobotConfig& config, std::shared_ptr<const Module> module);
    bool remove_robot(RobotId robot);
    StepResult step(const WorldInput& input = {});
    RobotSnapshot inspect(RobotId robot) const;
    std::span<const uint8_t> ram(RobotId robot) const;
private:
    struct Robot {
        Robot(const RobotConfig& config, std::shared_ptr<const Module> module);
        const RobotConfig config;
        Vm vm;
        RobotSnapshot stats;
        uint64_t remainder = 0, sequence = 0, deadline = 0;
        uint32_t pending_bytes = 0, pending_direction = 0;
        bool waiting = false;
        std::optional<uint64_t> queued_alarm;
    };
    class WorkerPool;
    SimulatorConfig config_;
    std::unique_ptr<WorkerPool> workers_;
    uint64_t tick_ = 0, time_ns_ = 0, clock_remainder_ = 0;
    RobotId next_id_ = 1;
    std::map<RobotId, Robot> robots_;
    std::set<std::pair<uint64_t, RobotId>> alarms_;
};
}

