#include "robot_sim/simulation.hpp"
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <set>
#include <system_error>
#include <thread>

namespace robot_sim {
class Simulation::WorkerPool {
public:
    explicit WorkerPool(uint32_t count) {
        threads_.reserve(count - 1);
        for (uint32_t i = 1; i < count; ++i) {
            try {
                threads_.emplace_back([this](std::stop_token stop) { worker(stop); });
            } catch (const std::system_error&) {
                break;
            }
        }
    }

    void run(const std::function<void()>& task, size_t jobs) {
        if (jobs <= 1 || threads_.empty()) {
            task();
            return;
        }
        {
            std::lock_guard lock(mutex_);
            task_ = &task;
            outstanding_ = threads_.size();
            ++generation_;
        }
        ready_.notify_all();
        task();
        std::unique_lock lock(mutex_);
        done_.wait(lock, [&] { return !outstanding_; });
    }

private:
    void worker(std::stop_token stop) {
        uint64_t seen = 0;
        std::unique_lock lock(mutex_);
        while (ready_.wait(lock, stop, [&] { return generation_ != seen; })) {
            if (stop.stop_requested()) return;
            seen = generation_;
            const auto* task = task_;
            lock.unlock();
            (*task)();
            lock.lock();
            if (!--outstanding_) done_.notify_one();
        }
    }

    std::mutex mutex_;
    std::condition_variable_any ready_;
    std::condition_variable done_;
    uint64_t generation_ = 0;
    size_t outstanding_ = 0;
    const std::function<void()>* task_ = nullptr;
    std::vector<std::jthread> threads_;
};

Simulation::Simulation(SimulatorConfig config) : config_(config) {
    if (!config.workers || config.workers > 64 ||
        !config.ticks_per_second || config.ticks_per_second > 1000000000)
        throw std::invalid_argument("invalid worker count or tick rate");
    workers_ = std::make_unique<WorkerPool>(config.workers);
}

Simulation::~Simulation() = default;

Simulation::Robot::Robot(const RobotConfig& cfg, std::shared_ptr<const Module> module)
    : config(cfg), vm(std::move(module), cfg.ram_pages) {
    vm.bus().configure(cfg.controller);
}

RobotId Simulation::add_robot(const RobotConfig& config, std::shared_ptr<const Module> module) {
    if (!config.active_cu_per_second || config.low_power_cu_per_second > config.active_cu_per_second ||
        !config.low_power_cu_per_second || config.active_cu_per_second > UINT64_MAX - config_.ticks_per_second ||
        config.ram_pages > 16384 || robots_.size() >= 1024 || next_id_ == UINT64_MAX)
        throw std::invalid_argument("invalid robot limits");
    robots_.try_emplace(next_id_, config, std::move(module));
    return next_id_++;
}

bool Simulation::remove_robot(RobotId id) {
    const auto found = robots_.find(id);
    if (found == robots_.end()) return false;
    if (const auto alarm = found->second.queued_alarm) alarms_.erase({*alarm, id});
    robots_.erase(found);
    return true;
}

RobotSnapshot Simulation::inspect(RobotId id) const {
    const auto& robot = robots_.at(id);
    const auto& bus = robot.vm.bus();
    auto snapshot = robot.stats;
    snapshot.power = bus.power();
    if (snapshot.state == RunState::sleeping && snapshot.power != Power::sleep)
        snapshot.state = RunState::budget;
    snapshot.instructions = robot.vm.instructions();
    snapshot.mmio_accesses = bus.mmio_accesses();
    snapshot.alarms = bus.alarm_count();
    snapshot.ram_pages = bus.pages();
    snapshot.pending_irq = bus.pending_irq();
    snapshot.status = bus.controller_status();
    snapshot.response = bus.controller_response();
    snapshot.rtc_ns = bus.rtc_ns();
    snapshot.location = robot.vm.location();
    return snapshot;
}

std::span<const uint8_t> Simulation::ram(RobotId id) const { return robots_.at(id).vm.bus().ram(); }

StepResult Simulation::step(const WorldInput& input) {
    const uint64_t nanos = 1000000000 + clock_remainder_;
    const auto elapsed = nanos / config_.ticks_per_second;
    if (elapsed > UINT64_MAX - time_ns_ || tick_ >= UINT64_MAX - UINT32_MAX)
        throw std::overflow_error("simulation clock exhausted");
    if (input.completions.size() > robots_.size() || input.irqs.size() > robots_.size())
        throw std::invalid_argument("too many boundary inputs");

    std::map<RobotId, uint32_t> irq_masks;
    for (const auto& irq : input.irqs) {
        if (!robots_.contains(irq.robot) || !irq_masks.emplace(irq.robot, irq.mask).second)
            throw std::invalid_argument("invalid or duplicate IRQ input");
    }
    std::set<RobotId> responders;
    for (const auto& response : input.completions) {
        const auto found = robots_.find(response.robot);
        if (found == robots_.end() || !responders.insert(response.robot).second ||
            !found->second.waiting || found->second.sequence != response.request ||
            (response.status != 2 && response.status != 3))
            throw std::invalid_argument("invalid completion input");
    }

    for (const auto& response : input.completions) {
        auto& robot = robots_.at(response.robot);
        const bool receive = robot.pending_direction == 1;
        const bool valid = response.status == 2 &&
            (receive ? response.payload.size() == robot.pending_bytes : response.payload.empty());
        robot.vm.bus().complete(valid ? 2 : 3, response.response, response.payload);
        robot.waiting = false;
        robot.stats.completed += valid;
        robot.stats.protocol_errors += !valid;
        if (valid) robot.stats.dma_bytes += robot.pending_bytes;
    }

    StepResult result{++tick_, time_ns_ += elapsed, 0, {}};
    clock_remainder_ = nanos % config_.ticks_per_second;
    std::vector<Robot*> runnable;
    runnable.reserve(robots_.size());
    result.commands.reserve(robots_.size());
    for (auto& [id, robot] : robots_) {
        auto& bus = robot.vm.bus();
        if (robot.waiting && tick_ >= robot.deadline) {
            bus.complete(4, 0);
            robot.waiting = false;
            ++robot.stats.protocol_errors;
        }
        const auto old_power = bus.power();
        const auto irq = irq_masks.find(id);
        bus.boundary(time_ns_ - elapsed, irq == irq_masks.end() ? 0 : irq->second, false);
        if (old_power == Power::sleep && bus.power() != Power::sleep) ++robot.stats.wakes;
        if (!robot.stats.fault.empty() || robot.stats.state == RunState::finished) continue;
        switch (bus.power()) {
        case Power::sleep:
            ++robot.stats.sleeping_ticks;
            robot.stats.sleeping_ns += elapsed;
            continue;
        case Power::active:
            ++robot.stats.active_ticks;
            robot.stats.active_ns += elapsed;
            break;
        case Power::low_power:
            ++robot.stats.low_power_ticks;
            robot.stats.low_power_ns += elapsed;
            break;
        }
        runnable.push_back(&robot);
    }
    std::atomic_size_t next{0};
    auto execute = [&] {
        for (auto index = next.fetch_add(1); index < runnable.size(); index = next.fetch_add(1)) {
            auto& robot = *runnable[index];
            const auto rate = robot.vm.bus().power() == Power::active ?
                robot.config.active_cu_per_second : robot.config.low_power_cu_per_second;
            const auto total = rate + robot.remainder;
            robot.remainder = total % config_.ticks_per_second;
            try {
                robot.stats.state = robot.vm.run(total / config_.ticks_per_second);
            } catch (const Fault& fault) {
                robot.stats.fault = fault.what();
                robot.stats.memory_fault = fault.kind;
                robot.stats.fault_address = fault.address;
            } catch (const std::exception& fault) {
                robot.stats.fault = fault.what();
            }
        }
    };
    workers_->run(execute, runnable.size());
    uint64_t hash = 14695981039346656037ull;
    auto mix = [&](uint64_t value) {
        for (int i = 0; i < 8; ++i) {
            hash = (hash ^ (value & 255)) * 1099511628211ull;
            value >>= 8;
        }
    };
    mix(tick_);
    mix(time_ns_);
    for (auto& [id, robot] : robots_) {
        const auto errors = robot.vm.bus().routing_errors();
        if (auto transfer = robot.vm.bus().take_request()) {
            ++robot.stats.transactions;
            robot.waiting = true;
            robot.deadline = tick_ + robot.config.controller->timeout_ticks;
            robot.pending_bytes = transfer->dma_size;
            robot.pending_direction = transfer->direction;
            result.commands.push_back({id, ++robot.sequence, std::move(*transfer)});
        }
        const auto rejected = robot.vm.bus().routing_errors() - errors;
        robot.stats.protocol_errors += rejected;
        robot.stats.transactions += rejected;
        const auto alarm = robot.vm.bus().alarm_deadline();
        if (alarm != robot.queued_alarm) {
            if (robot.queued_alarm) alarms_.erase({*robot.queued_alarm, id});
            if (alarm) alarms_.emplace(*alarm, id);
            robot.queued_alarm = alarm;
        }
    }
    while (!alarms_.empty() && alarms_.begin()->first <= time_ns_) {
        const auto id = alarms_.begin()->second;
        alarms_.erase(alarms_.begin());
        auto& robot = robots_.at(id);
        robot.queued_alarm.reset();
        const auto previous_power = robot.vm.bus().power();
        robot.vm.bus().boundary(time_ns_, 0);
        if (previous_power == Power::sleep && robot.vm.bus().power() != Power::sleep)
            ++robot.stats.wakes;
    }
    for (auto& [id, robot] : robots_) {
        auto& bus = robot.vm.bus();
        const auto previous_power = bus.power();
        bus.boundary(time_ns_, 0, false);
        if (previous_power == Power::sleep && bus.power() != Power::sleep) ++robot.stats.wakes;
        mix(id);
        mix(robot.remainder);
        mix(robot.sequence);
        mix(robot.waiting ? robot.deadline : 0);
        mix(robot.vm.checksum());
    }
    result.checksum = hash;
    return result;
}
}

