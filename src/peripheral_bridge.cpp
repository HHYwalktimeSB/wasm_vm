#include "robot_sim/peripheral_bridge.hpp"
#include "checksum.hpp"
#include <algorithm>
#include <tuple>
#include <utility>

namespace robot_sim {
namespace {
constexpr size_t max_inputs = 1024;

bool valid_pins(Pins pins) {
    return pins.clock < 32 && pins.data < 32 && pins.clock != pins.data;
}
bool same_pins(Pins first, Pins second) {
    return first.clock == second.clock && first.data == second.data;
}
}

PeripheralBridge::PeripheralBridge(SimulatorConfig config)
    : simulation_(config), tick_rate_(config.ticks_per_second) {}

RobotId PeripheralBridge::add_robot(const RobotConfig& config, std::shared_ptr<const Module> module) {
    const auto id = simulation_.add_robot(config, std::move(module));
    try {
        connections_.emplace(id, Connection{config.controller});
    } catch (...) {
        simulation_.remove_robot(id);
        throw;
    }
    return id;
}

bool PeripheralBridge::remove_robot(RobotId id) {
    if (!connections_.contains(id)) return false;
    for (auto it = pending_.begin(); it != pending_.end(); ++it) {
        if (it->second.robot != id) continue;
        removed_.push_back({it->first, id, it->second.device, TransactionOutcome::cancelled});
        release(it->first, it->second);
        pending_.erase(it);
        break;
    }
    std::erase_if(route_failures_, [=](const auto& failure) { return failure.robot == id; });
    connections_.erase(id);
    return simulation_.remove_robot(id);
}

DeviceId PeripheralBridge::register_device(PeripheralDevice config) {
    if (config.protocol != Protocol::abstract || !valid_pins(config.pins) ||
        config.max_payload > 65536 || !config.bytes_per_second ||
        devices_.size() >= 1024 || next_device_ == UINT64_MAX)
        throw std::invalid_argument("invalid peripheral capability");
    devices_.emplace(next_device_, Device{config});
    return next_device_++;
}

void PeripheralBridge::release(TxId id, const InFlight& transfer) {
    if (auto device = devices_.find(transfer.device);
        device != devices_.end() && device->second.active == id)
        device->second.active = 0;
}

void PeripheralBridge::disconnect(DeviceId id) {
    auto& device = devices_.at(id);
    device.config.available = false;
    if (device.active) pending_.at(device.active).cancelled = true;
}

bool PeripheralBridge::remove_device(DeviceId id) {
    if (!devices_.contains(id)) return false;
    disconnect(id);
    devices_.erase(id);
    std::erase_if(events_, [=](const auto& event) { return event.device == id; });
    for (auto& [robot, connection] : connections_)
        if (connection.device == id) connection.device = 0;
    return true;
}

void PeripheralBridge::connect(RobotId robot, DeviceId device) {
    auto& connection = connections_.at(robot);
    if (!connection.controller || !devices_.contains(device))
        throw std::invalid_argument("missing controller or device");
    if (simulation_.inspect(robot).status == 1)
        throw std::logic_error("cannot rewire a busy controller");
    connection.device = device;
}

bool PeripheralBridge::cancel(TxId id) {
    const auto transfer = pending_.find(id);
    if (transfer == pending_.end() || transfer->second.cancelled) return false;
    transfer->second.cancelled = true;
    return true;
}

PeripheralStep PeripheralBridge::step(PeripheralInput input) {
    const Tick boundary = tick_ + 1;
    PeripheralStep output{boundary, 0, 0, {}, std::move(removed_), {}};
    removed_.clear();
    WorldInput world;
    world.completions.swap(route_failures_);
    if (input.responses.size() > max_inputs || input.events.size() > max_inputs) {
        output.rejected.push_back({0, BridgeError::input_limit});
        input = {};
    }

    for (auto& reply : input.responses) {
        const auto found = pending_.find(reply.id);
        if (found == pending_.end() || found->second.cancelled) {
            output.rejected.push_back({reply.id, BridgeError::unknown_transaction});
            continue;
        }
        auto& transfer = found->second;
        if (transfer.response) {
            output.rejected.push_back({reply.id, BridgeError::duplicate_response});
            continue;
        }
        const bool success = reply.result == HostResult::success;
        const bool known_result = success || reply.result == HostResult::busy ||
            reply.result == HostResult::device_error || reply.result == HostResult::no_response;
        if (!known_result || reply.earliest_ready_tick < transfer.accepted ||
            reply.rx.size() != (success ? transfer.expected_rx : 0)) {
            output.rejected.push_back({reply.id, BridgeError::invalid_response});
            continue;
        }
        reply.earliest_ready_tick = std::max(boundary, reply.earliest_ready_tick);
        transfer.response = std::move(reply);
    }

    for (const auto& event : input.events) {
        auto found = devices_.find(event.device);
        if (found == devices_.end()) {
            output.rejected.push_back({event.device, BridgeError::unknown_device});
        } else if ((event.kind != PeripheralEventKind::data_ready &&
                    event.kind != PeripheralEventKind::disconnected) ||
                   event.earliest_delivery_tick < boundary || event.host_seq <= found->second.last_event) {
            output.rejected.push_back({event.device, BridgeError::invalid_event});
        } else if (events_.size() >= max_inputs) {
            output.rejected.push_back({event.device, BridgeError::input_limit});
        } else {
            events_.push_back(event);
            found->second.last_event = event.host_seq;
        }
    }
    std::ranges::sort(events_, {}, [](const auto& event) {
        return std::tuple{event.earliest_delivery_tick, event.device, event.host_seq};
    });
    std::set<RobotId> notified;
    for (const auto& event : events_) {
        if (event.earliest_delivery_tick > boundary) break;
        if (event.kind == PeripheralEventKind::disconnected) {
            disconnect(event.device);
        } else {
            const auto& device = devices_.at(event.device).config;
            if (device.available && device.data_ready_irq)
                for (const auto& [robot, connection] : connections_)
                    if (connection.device == event.device && connection.controller &&
                        same_pins(connection.controller->controller, device.pins) &&
                        same_pins(connection.controller->peripheral, device.pins) &&
                        connection.controller->address == device.address)
                        notified.insert(robot);
        }
    }
    std::erase_if(events_, [=](const auto& event) { return event.earliest_delivery_tick <= boundary; });
    for (auto robot : notified) world.irqs.push_back({robot, data_ready_irq});

    for (auto it = pending_.begin(); it != pending_.end();) {
        auto& [id, transfer] = *it;
        std::optional<TransactionOutcome> outcome;
        if (transfer.cancelled) {
            outcome = TransactionOutcome::cancelled;
        } else if (transfer.response && transfer.response->earliest_ready_tick <= boundary) {
            const auto result = transfer.response->result;
            if (result == HostResult::success) {
                transfer.credits += transfer.rate;
                if (transfer.credits >= uint64_t(transfer.wire_bytes) * tick_rate_)
                    outcome = TransactionOutcome::completed;
            } else if (result != HostResult::no_response) {
                outcome = TransactionOutcome::failed;
            }
        }
        if (!outcome && boundary >= transfer.deadline) outcome = TransactionOutcome::timed_out;
        if (!outcome) {
            ++it;
            continue;
        }
        if (*outcome == TransactionOutcome::completed) {
            world.completions.push_back({transfer.robot, transfer.sequence, 2,
                transfer.response->response, std::move(transfer.response->rx)});
        } else if (*outcome != TransactionOutcome::timed_out) {
            world.completions.push_back({transfer.robot, transfer.sequence, 3, 0, {}});
        }
        output.terminal.push_back({id, transfer.robot, transfer.device, *outcome});
        release(id, transfer);
        it = pending_.erase(it);
    }

    auto step = simulation_.step(world);
    tick_ = step.tick;
    output.time_ns = step.time_ns;
    for (auto& [robot, sequence, transfer] : step.commands) {
        const auto& connection = connections_.at(robot);
        const auto& controller = *connection.controller;
        const auto device = devices_.find(connection.device);
        constexpr uint32_t word_bytes = 4;
        const uint32_t wire_bytes = transfer.direction ? transfer.dma_size : word_bytes * 2;
        if (device == devices_.end() || !device->second.config.available || device->second.active ||
            device->second.config.protocol != controller.protocol ||
            !same_pins(controller.peripheral, device->second.config.pins) ||
            transfer.target != device->second.config.address ||
            (transfer.direction ? transfer.dma_size : word_bytes) > device->second.config.max_payload ||
            next_tx_ == UINT64_MAX) {
            route_failures_.push_back({robot, sequence, 3, 0, {}});
            continue;
        }
        const auto id = next_tx_++;
        const auto expected = transfer.direction == 1 ? transfer.dma_size : 0;
        pending_.emplace(id, InFlight{robot, device->first, sequence, tick_,
            tick_ + controller.timeout_ticks, expected, wire_bytes,
            std::min(controller.bytes_per_second, device->second.config.bytes_per_second)});
        device->second.active = id;
        const auto direction = transfer.direction == 1 ? TransferKind::read :
                               transfer.direction == 2 ? TransferKind::write : TransferKind::command;
        output.requests.push_back({id, robot, 0, device->first, controller.protocol, direction,
            AbstractParams{transfer.target}, transfer.command, expected, tick_, sequence,
            std::move(transfer.payload)});
    }
    std::ranges::sort(output.terminal, {}, &TransactionEnd::id);

    Checksum hash;
    hash.word(step.checksum);
    hash.word(next_device_);
    hash.word(next_tx_);
    for (const auto& [robot, connection] : connections_) {
        hash.word(robot);
        hash.word(connection.device);
    }
    for (const auto& [device, info] : devices_) {
        hash.word(device);
        hash.word(info.config.available);
        hash.word(info.active);
        hash.word(info.last_event);
    }
    for (const auto& [id, transfer] : pending_) {
        hash.word(id);
        hash.word(transfer.robot);
        hash.word(transfer.device);
        hash.word(transfer.credits);
        hash.word(transfer.response.has_value());
        if (transfer.response) {
            hash.word(transfer.response->earliest_ready_tick);
            hash.word(static_cast<uint32_t>(transfer.response->result));
            hash.word(transfer.response->response);
            hash.word(transfer.response->rx.size());
            for (auto byte : transfer.response->rx) hash.byte(byte);
        }
    }
    for (const auto& event : events_) {
        hash.word(event.device);
        hash.word(static_cast<uint32_t>(event.kind));
        hash.word(event.host_seq);
        hash.word(event.earliest_delivery_tick);
    }
    for (const auto& failure : route_failures_) hash.word(failure.robot);
    output.checksum = hash.value();
    return output;
}
}

