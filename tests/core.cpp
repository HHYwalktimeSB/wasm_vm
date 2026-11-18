#include "robot_sim/vm.hpp"
#include "robot_sim/simulation.hpp"
#include <initializer_list>
#include <fstream>
#include <iostream>

using namespace robot_sim;

static void check(bool ok) {
    if (!ok) throw std::runtime_error("test assertion failed");
}
static void expect_fault(auto action, FaultKind kind) {
    try { action(); }
    catch (const Fault& fault) { check(fault.kind == kind); return; }
    throw std::runtime_error("expected memory fault");
}

static auto fixture_bytes(std::initializer_list<uint8_t> code) {
    std::vector<uint8_t> module{
        0,97,115,109,1,0,0,0,
        1,4,1,0x60,0,0,
        3,2,1,0,
        5,4,1,1,1,2,
        7,17,1,13,'r','o','b','o','t','_','m','a','i','n','_','v','1',0,0,
        10,uint8_t(code.size()+3),1,uint8_t(code.size()+1),0};
    module.insert(module.end(), code);
    return module;
}

static auto fixture(std::initializer_list<uint8_t> code) { return validate(fixture_bytes(code)); }

static std::vector<uint64_t> simulation_test(uint32_t workers) {
    const auto module = fixture({
        0x41,0x80,0x82,0x80,0x80,4,0x41,7,0x36,2,0,
        0x41,0xa0,0x82,0x80,0x80,4,0x41,16,0x36,2,0,
        0x41,0xa4,0x82,0x80,0x80,4,0x41,2,0x36,2,0,
        0x41,0xa8,0x82,0x80,0x80,4,0x41,1,0x36,2,0,
        0x41,0xa4,0x80,0x80,0x80,4,0x41,4,0x36,2,0,
        0x41,0x80,0x80,0x80,0x80,4,0x41,2,0x36,2,0,
        0x41,16,0x41,5,0x3a,0,0,0x0b});
    Simulation simulation({10, workers});
    RobotConfig config{1000, 250, 1, ControllerConfig{}};
    config.controller->dma_max_bytes = 16;
    for (unsigned i = 0; i < 100; ++i) simulation.add_robot(config, module);
    const auto first = simulation.step();
    check(first.commands.size() == 100 && first.time_ns == 100000000);
    for (size_t i = 0; i < first.commands.size(); ++i) {
        check(first.commands[i].robot == i + 1);
        check(first.commands[i].transfer.dma_address == 16);
        check(simulation.inspect(i + 1).power == Power::sleep);
    }
    const auto idle = simulation.step();
    check(idle.commands.empty() && simulation.inspect(1).sleeping_ticks == 1);
    const auto asleep_instructions = simulation.inspect(1).instructions;
    WorldInput input;
    input.completions.push_back({1, first.commands.front().request, 2, 0, {33, 44}});
    const auto completed = simulation.step(input);
    const auto state = simulation.inspect(1);
    check(state.pending_irq == 4 && state.wakes == 1 && state.dma_bytes == 2);
    check(state.rtc_ns == completed.time_ns && state.state == RunState::finished);
    check(simulation.ram(1)[16] == 5 && simulation.ram(1)[17] == 44);
    check(simulation.inspect(2).instructions == asleep_instructions && simulation.ram(2)[17] == 0);
    WorldInput denied;
    denied.completions.push_back({2, first.commands[1].request, 3, 0, {}});
    const auto denied_step = simulation.step(denied);
    check(simulation.inspect(2).status == 3 && simulation.inspect(2).wakes == 1);
    check(simulation.inspect(2).dma_bytes == 0 && simulation.ram(2)[17] == 0);
    return {first.checksum, idle.checksum, completed.checksum, denied_step.checksum};
}

static void device_boundaries() {
    Bus bus(1, 1);
    ControllerConfig controller;
    controller.dma_max_bytes = 8;
    bus.configure(controller);
    const auto base = bus.base();
    bus.store(base + 0x100, 4, 7);
    bus.store(base + 0x120, 4, Bus::page_size - 1);
    bus.store(base + 0x124, 4, 2);
    expect_fault([&] { bus.store(base + 0x128, 4, 1); }, FaultKind::bounds);
    bus.store(base + 0x120, 4, 16);
    bus.store(16, 2, 0x2211);
    bus.store(base + 0x128, 4, 2);
    const auto transmit = bus.take_request();
    check(transmit && transmit->payload == std::vector<uint8_t>({0x11, 0x22}));
    bus.complete(2, 0);
    check(bus.pending_irq() == 4);
    bus.store(base + 0x20, 4, 4);
    bus.store(base + 0x18, 4, 1);
    bus.store(base + 0x28, 4, 1);
    bus.store(base + 0x24, 4, 2);
    bus.store(base, 4, 2);
    bus.boundary(0, 0);
    check(bus.power() == Power::sleep);
    bus.boundary(10, 0);
    check(bus.power() == Power::active && bus.pending_irq() == 2 && bus.alarm_count() == 1);
    bus.store(base, 4, 2);
    bus.boundary(20, 0);
    check(bus.power() == Power::active && bus.pending_irq() == 2);
}

static void scheduling_limits() {
    Simulation simulation({1, 2});
    RobotConfig config{10, 4, 1, ControllerConfig{}};
    config.controller->timeout_ticks = 1;
    const auto toggle = simulation.add_robot(config, fixture({
        0x41,0x80,0x80,0x80,0x80,4,0x41,0,0x36,2,0,
        0x41,0x80,0x80,0x80,0x80,4,0x41,1,0x36,2,0,
        0x03,0x40,0x0c,0,0x0b,0x0b}));
    const auto trap = simulation.add_robot(config, fixture({0x00,0x0b}));
    const auto send = simulation.add_robot(config, fixture({
        0x41,0x80,0x82,0x80,0x80,4,0x41,7,0x36,2,0,
        0x41,0x84,0x82,0x80,0x80,4,0x41,42,0x36,2,0,0x0b}));
    const auto first = simulation.step();
    check(first.commands.size() == 1 && first.commands[0].robot == send);
    check(simulation.inspect(toggle).instructions == 10 && simulation.inspect(toggle).power == Power::low_power);
    check(!simulation.inspect(trap).fault.empty());
    const auto second = simulation.step();
    check(second.commands.empty() && simulation.inspect(toggle).instructions == 14);
    check(simulation.inspect(send).status == 4 && simulation.inspect(send).protocol_errors == 1);
    check(simulation.inspect(send).pending_irq == 1 && simulation.inspect(trap).instructions == 1);
}

int main(int argc, char** argv) {
    if (argc == 2) {
        const auto module = fixture_bytes({
            0x41,0x80,0x82,0x80,0x80,4,0x41,7,0x36,2,0,
            0x41,0x84,0x82,0x80,0x80,4,0x41,42,0x36,2,0,
            0x41,0x98,0x80,0x80,0x80,4,0x41,1,0x36,2,0,
            0x41,0xa8,0x80,0x80,0x80,4,0x41,1,0x36,2,0,
            0x41,0xa4,0x80,0x80,0x80,4,0x41,2,0x36,2,0,
            0x41,0x80,0x80,0x80,0x80,4,0x41,2,0x36,2,0,0x0b});
        std::ofstream output(argv[1], std::ios::binary);
        output.write(reinterpret_cast<const char*>(module.data()), module.size());
        output.close();
        return output ? 0 : 1;
    }
    Bus bus(1, 2);
    for (unsigned width : {1, 2, 4, 8}) {
        const auto value = UINT64_MAX >> (64 - width * 8);
        bus.store(3, width, value);
        check(bus.load(3, width) == value);
    }
    bus.store(3, 4, 0x12345678);
    check(bus.load(3, 1) == 0x78 && bus.load(6, 1) == 0x12);
    bus.store(65535, 1, 42);
    expect_fault([&] { bus.store(65535, 2, 0); }, FaultKind::bounds);
    check(bus.load(65535, 1) == 42);
    expect_fault([&] { bus.load(0xffffffff, 4); }, FaultKind::unmapped);
    expect_fault([&] { bus.store(bus.base() + 1, 4, 0); }, FaultKind::alignment);
    expect_fault([&] { bus.store(bus.base(), 2, 0); }, FaultKind::width);
    check(bus.grow(1) == 1 && bus.load(65536, 8) == 0 && bus.grow(1) == UINT32_MAX);
    try { bus.connect({0, 0}, {0, 1}, 7); check(false); }
    catch (const std::invalid_argument&) {}
    bus.connect({0, 1}, {1, 0}, 7);
    bus.store(bus.base() + 0x100, 4, 7);
    bus.store(bus.base() + 0x104, 4, 42);
    check(bus.load(bus.base() + 0x108, 4) == 1);
    bus.tick();
    check(bus.load(bus.base() + 0x108, 4) == 3);
    bus.connect({0, 1}, {0, 1}, 7);
    bus.store(bus.base() + 0x104, 4, 42);
    bus.tick();
    check(bus.load(bus.base() + 0x108, 4) == 2 && bus.load(bus.base() + 0x10c, 4) == (42 ^ 0xa5));

    Vm loop(fixture({0x03,0x40,0x0c,0,0x0b,0x0b}));
    check(loop.run(10) == RunState::budget && loop.instructions() == 10);
    check(loop.run(11) == RunState::budget && loop.instructions() == 21);
    loop.bus().store(loop.bus().base(), 4, 1);
    check(loop.step(5) == RunState::budget && loop.instructions() == 23);
    check(loop.step(5) == RunState::budget && loop.instructions() == 26);
    try { fixture({0x6a,0x0b}); check(false); }
    catch (const LoadError&) {}
    try { fixture({0xfd,0x0b}); check(false); }
    catch (const LoadError&) {}

    const auto extension = fixture_bytes({0x41,0,0x41,0xff,1,0xc0,0x36,2,0,0x0b});
    try { validate(extension); check(false); }
    catch (const LoadError&) {}
    Vm extended(validate(extension, {.sign_extension = true}));
    check(extended.run(20) == RunState::finished && extended.bus().load(0,4) == UINT32_MAX);
    const auto growth = fixture({
        0x41,0,0x41,1,0x40,0,0x36,2,0,
        0x41,4,0x41,1,0x40,0,0x36,2,0,0x0b});
    Vm growing(growth, 2);
    check(growing.run(20) == RunState::finished && growing.bus().pages() == 2);
    check(growing.bus().load(0,4) == 1 && growing.bus().load(4,4) == UINT32_MAX);
    Vm capped(growth, 1);
    check(capped.run(20) == RunState::finished && capped.bus().pages() == 1);
    check(capped.bus().load(0,4) == UINT32_MAX);

    auto arithmetic = fixture({0x41,0,0x41,6,0x41,7,0x6c,0x36,2,0,0x0b});
    Vm stepped(arithmetic), straight(arithmetic);
    check(stepped.run(3) == RunState::budget && stepped.bus().load(0,4) == 0);
    check(stepped.run(3) == RunState::finished && straight.run(6) == RunState::finished);
    check(stepped.bus().load(0,4) == 42 && straight.bus().load(0,4) == 42);

    auto power = fixture({
        0x41,0x80,0x80,0x80,0x80,4,0x41,1,0x36,2,0,
        0x41,0x80,0x80,0x80,0x80,4,0x41,2,0x36,2,0,
        0x41,0,0x41,42,0x36,2,0,0x0b});
    Vm sleeper(power);
    check(sleeper.run(3) == RunState::budget && sleeper.bus().power() == Power::low_power);
    check(sleeper.run(20) == RunState::sleeping && sleeper.instructions() == 6);
    check(sleeper.run(20) == RunState::sleeping && sleeper.bus().load(0,4) == 0);
    sleeper.bus().store(sleeper.bus().base(), 4, 0);
    check(sleeper.run(20) == RunState::finished && sleeper.bus().load(0,4) == 42);

    Vm transfer(fixture({
        0x41,0x80,0x82,0x80,0x80,4,0x41,7,0x36,2,0,
        0x41,0x84,0x82,0x80,0x80,4,0x41,42,0x36,2,0,0x0b}));
    transfer.bus().connect({0, 1}, {0, 1}, 7);
    check(transfer.run(20) == RunState::finished);
    check(transfer.bus().load(0x40000108, 4) == 1);
    check(transfer.bus().load(0x4000010c, 4) == 0);
    transfer.bus().tick();
    check(transfer.bus().load(0x40000108, 4) == 2);
    check(transfer.bus().load(0x4000010c, 4) == 143);
    check(transfer.bus().load(0x104, 4) == 0);

    Vm trapped(fixture({0x41,0x7f,0x28,2,1,0x1a,0x0b}));
    expect_fault([&] { trapped.run(20); }, FaultKind::unmapped);
    device_boundaries();
    scheduling_limits();
    const auto reference = simulation_test(1);
    check(simulation_test(2) == reference && simulation_test(4) == reference);
    expect_fault([&] { trapped.run(20); }, FaultKind::unmapped);
    std::cout << "core acceptance tests passed\n";
}

