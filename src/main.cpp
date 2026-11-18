#include "robot_sim/vm.hpp"
#include "cli_scenario.hpp"
#include "checksum.hpp"
#include <charconv>
#include <fstream>
#include <iostream>
#include <string_view>

using namespace robot_sim;

namespace {
constexpr auto usage = "robot_sim_cli validate MODULE\n"
                       "robot_sim_cli run MODULE [TICKS [CU_PER_TICK [RAM_PAGES]]]\n"
                       "robot_sim_cli {run|profile|test} MODULE --ticks N [OPTIONS]\n"
                       "  --workers N --tick-rate N --active-cu N --low-cu N --ram-pages N\n"
                       "  --dma-bytes N --seed N --trace PATH --replay PATH --json PATH\n"
                       "  --robot PATH --scenario PATH\n"
                       "  --expect-status N --expect-wakes N --expect-commands N\n"
                       "  --expect-alarms N --expect-power N --expect-trap N\n";

uint32_t number(std::string_view text) {
    uint32_t value = 0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size() || !value)
        throw std::invalid_argument("expected positive integer");
    return value;
}

std::shared_ptr<const Module> read_module(const char* path, Checksum& hash) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    const auto size = input.tellg();
    if (size < 0 || size > 4 * 1024 * 1024) throw LoadError("cannot open module or module too large");
    std::vector<uint8_t> data(static_cast<size_t>(size));
    input.seekg(0);
    if (!input.read(reinterpret_cast<char*>(data.data()), size)) throw LoadError("cannot read module");
    for (auto byte : data) hash.byte(byte);
    return validate(data);
}
}

int main(int argc, char** argv) {
    const std::string_view command = argc > 1 ? argv[1] : "--help";
    if (command == "--help" && argc <= 2) {
        std::cout << usage;
        return 0;
    }
    const bool named = argc > 3 && std::string_view(argv[2]) == "--program";
    const int program = named ? 3 : 2;
    const int options = program + 1;
    const bool modern = command == "profile" || command == "test" ||
        (command == "run" && named) ||
        (command == "run" && argc > 3 && std::string_view(argv[3]).starts_with("--"));
    if (argc <= program || (!modern && (command == "validate" ? argc != options : command != "run" || argc > 6))) {
        std::cerr << usage;
        return 2;
    }
    try {
        Checksum hash;
        auto module = read_module(argv[program], hash);
        if (modern) return run_scenario(command, {argv + options, static_cast<size_t>(argc - options)},
                                        std::move(module), hash.value());
        if (command == "validate") {
            std::cout << "valid robot-wasm32-v1 module\n";
            return 0;
        }
        const auto ticks = argc > 3 ? number(argv[3]) : 1;
        const auto units = argc > 4 ? number(argv[4]) : 1000;
        const auto cap = argc > 5 ? number(argv[5]) : 16;
        Vm vm(module, cap);
        vm.bus().connect({0, 1}, {0, 1}, 7);
        auto state = RunState::budget;
        uint32_t elapsed = 0;
        while (elapsed < ticks && state != RunState::finished) {
            state = vm.step(units);
            ++elapsed;
        }
        std::cout << "ticks=" << elapsed << " instructions=" << vm.instructions()
                  << " state=" << (state == RunState::finished ? "finished" :
                      vm.bus().power() == Power::sleep ? "sleeping" : "budget")
                  << " status=" << vm.bus().load(vm.bus().base() + 0x108, 4)
                  << " response=" << vm.bus().load(vm.bus().base() + 0x10c, 4) << '\n';
    } catch (const LoadError& error) {
        std::cerr << "load error: " << error.what() << '\n';
        return 3;
    } catch (const std::invalid_argument& error) {
        std::cerr << error.what() << '\n';
        return 2;
    } catch (const std::exception& error) {
        std::cerr << "trap: " << error.what() << '\n';
        return 4;
    }
}

