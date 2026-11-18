#include "robot_sim/peripheral_bridge.hpp"
#include <fstream>
#include <iostream>

using namespace robot_sim;

namespace {
void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

std::shared_ptr<const Module> fixture() {
    // Write target, read it back, then submit command 42.
    const std::vector<uint8_t> code{
        0,
        0x41,0x80,0x82,0x80,0x80,4,0x41,7,0x36,2,0,
        0x41,0x80,0x82,0x80,0x80,4,0x28,2,0,0x41,7,0x47,0x04,0x40,0x00,0x0b,
        0x41,0x84,0x82,0x80,0x80,4,0x41,42,0x36,2,0,0x0b};
    std::vector<uint8_t> bytes{
        0,97,115,109,1,0,0,0,
        1,4,1,0x60,0,0,3,2,1,0,5,3,1,0,1,
        7,17,1,13,'r','o','b','o','t','_','m','a','i','n','_','v','1',0,0,
        10,uint8_t(code.size()+2),1,uint8_t(code.size())};
    bytes.insert(bytes.end(), code.begin(), code.end());
    return validate(bytes);
}

PeripheralResponse mock_reply(const PeripheralRequest& request, Tick ready) {
    check(request.controller == 0 && request.protocol == Protocol::abstract &&
          request.direction == TransferKind::command && request.command == 42 &&
          std::get<AbstractParams>(request.params).address == 7, "wrong normalized mock request");
    return {request.id, HostResult::success, ready, request.command ^ 0xa5u, {}};
}

std::vector<uint64_t> run(uint32_t workers) {
    PeripheralBridge bridge({10, workers});
    PeripheralDevice fast;
    fast.data_ready_irq = true;
    const auto shared = bridge.register_device(fast);
    PeripheralDevice slow;
    slow.bytes_per_second = 40; // Four bytes per tick. Command and response are eight bytes.
    const auto delayed = bridge.register_device(slow);
    auto bad_device = fast;
    bad_device.pins = {1, 0};
    const auto unwired = bridge.register_device(bad_device);

    RobotConfig config{1000, 250, 1, ControllerConfig{}};
    const auto program = fixture();
    const auto first = bridge.add_robot(config, program);
    const auto second = bridge.add_robot(config, program);
    const auto third = bridge.add_robot(config, program);
    const auto fourth = bridge.add_robot(config, program);
    bridge.connect(first, shared);
    bridge.connect(second, shared);
    bridge.connect(third, delayed);
    bridge.connect(fourth, unwired);
    const auto a = bridge.step();
    check(a.requests.size() == 2 && a.requests[0].robot == first && a.requests[1].robot == third,
          "unstable shared-device arbitration or invalid route forwarded");
    check(a.requests[0].id != a.requests[1].id && bridge.inspect(first).status == 1 &&
          !bridge.inspect(first).pending_irq && bridge.inspect(first).fault.empty(),
          "request unexpectedly complete or local MMIO mismatch");

    auto response = mock_reply(a.requests[0], 1); // Same host-ready tick, visible at tick 2.
    const auto b = bridge.step({{response, response, mock_reply(a.requests[1], 4)},
                              {{shared, PeripheralEventKind::data_ready, 3, 1},
                               {delayed, PeripheralEventKind::data_ready, 3, 1}}});
    check(b.rejected.size() == 1 && b.rejected[0].error == BridgeError::duplicate_response,
          "duplicate result was accepted");
    check(bridge.inspect(first).status == 2 && bridge.inspect(first).response == 143 &&
          bridge.inspect(second).status == 3 && bridge.inspect(fourth).status == 3,
          "incorrect success, arbitration or pin failure");
    const auto c = bridge.step({{response, {999, HostResult::success, 3}}, {}});
    check(c.rejected.size() == 2 && c.rejected[0].error == BridgeError::unknown_transaction,
          "stale or unknown response accepted");
    check((bridge.inspect(first).pending_irq & PeripheralBridge::data_ready_irq) &&
          !(bridge.inspect(third).pending_irq & PeripheralBridge::data_ready_irq),
          "data-ready IRQ did not follow capability");
    const auto d = bridge.step();
    check(bridge.inspect(third).status == 1 && !bridge.inspect(third).pending_irq,
          "bandwidth or readiness ignored");
    const auto e = bridge.step();
    check(bridge.inspect(third).status == 2 && e.terminal.size() == 1 &&
          e.terminal[0].outcome == TransactionOutcome::completed, "incorrect delayed completion boundary");
    return {a.checksum, b.checksum, c.checksum, d.checksum, e.checksum};
}

void cancellation() {
    PeripheralBridge bridge({10, 1});
    const auto device = bridge.register_device({});
    RobotConfig config{1000, 250, 1, ControllerConfig{}};
    config.controller->timeout_ticks = 2;
    const auto program = fixture();
    auto start = [&] {
        const auto robot = bridge.add_robot(config, program);
        bridge.connect(robot, device);
        const auto start = bridge.step();
        check(start.requests.size() == 1, "expected accepted transaction");
        return start.requests[0];
    };
    const auto first = start();
    check(bridge.cancel(first.id) && !bridge.cancel(first.id), "expected one cancel");
    const auto cancelled = bridge.step({{mock_reply(first, first.accepted_tick)}, {}});
    check(cancelled.terminal.size() == 1 &&
          cancelled.terminal[0].outcome == TransactionOutcome::cancelled &&
          cancelled.rejected.size() == 1 && bridge.inspect(first.robot).status == 3,
          "cancel did not reject late response");

    const auto second = start();
    check(bridge.remove_robot(second.robot), "remove active VM failed");
    const auto removed = bridge.step({{mock_reply(second, second.accepted_tick)}, {}});
    check(removed.terminal.size() == 1 && removed.rejected.size() == 1,
          "removed VM retained a transaction");

    const auto third = start();
    check(bridge.remove_device(device), "remove active device failed");
    const auto unplugged = bridge.step({{mock_reply(third, third.accepted_tick)}, {}});
    check(unplugged.terminal.size() == 1 && unplugged.rejected.size() == 1 &&
          bridge.inspect(third.robot).status == 3, "unplug left transfer active");
    check(bridge.register_device({}) > device, "device ID reused");
}

void timeout_and_disconnect() {
    PeripheralBridge bridge({10, 1});
    const auto device = bridge.register_device({});
    RobotConfig config{1000, 250, 1, ControllerConfig{}};
    config.controller->timeout_ticks = 2;
    const auto program = fixture();
    const auto first = bridge.add_robot(config, program);
    bridge.connect(first, device);
    const auto sent = bridge.step().requests.at(0);
    const auto invalid = bridge.step({{{sent.id, HostResult::success, 2, 1, {4}}}, {}});
    check(invalid.rejected.size() == 1 && invalid.rejected[0].error == BridgeError::invalid_response &&
          bridge.inspect(first).status == 1, "invalid payload completed a pending transaction");
    const auto timed_out = bridge.step();
    check(timed_out.terminal.size() == 1 && timed_out.terminal[0].outcome == TransactionOutcome::timed_out &&
          bridge.inspect(first).status == 4, "timeout boundary did not release the device");

    const auto second = bridge.add_robot(config, program);
    bridge.connect(second, device);
    const auto again = bridge.step().requests.at(0);
    const auto disconnect = bridge.step({{mock_reply(again, 5)},
                                         {{device, PeripheralEventKind::disconnected, 5, 1}}});
    check(disconnect.terminal.size() == 1 &&
          disconnect.terminal[0].outcome == TransactionOutcome::cancelled &&
          bridge.inspect(second).status == 3, "disconnect lost priority over simultaneous reply");
}

void c_guest(const char* path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    const auto size = file.tellg();
    check(size > 0 && size <= 4 * 1024 * 1024, "invalid compiled guest file");
    std::vector<uint8_t> bytes(static_cast<size_t>(size));
    file.seekg(0);
    check(bool(file.read(reinterpret_cast<char*>(bytes.data()), size)), "could not read compiled guest");
    PeripheralBridge bridge({10, 2});
    const auto id = bridge.add_robot({1000, 250, 1, ControllerConfig{}}, validate(bytes));
    bridge.connect(id, bridge.register_device({}));
    const auto a = bridge.step();
    check(a.requests.size() == 1 && bridge.inspect(id).status == 1, "compiled guest request missing");
    bridge.step({{mock_reply(a.requests[0], 1)}, {}});
    check(bridge.inspect(id).state == RunState::finished && bridge.inspect(id).fault.empty(),
          "compiled guest did not read bridge response");
}
}

int main(int argc, char** argv) {
    try {
        if (argc == 2) {
            c_guest(argv[1]);
        } else {
            const auto reference = run(1);
            check(run(2) == reference && run(4) == reference, "worker counts changed the bridge trace");
            cancellation();
            timeout_and_disconnect();
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

