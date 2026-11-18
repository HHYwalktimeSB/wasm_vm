# Simulator design

CMake builds the C++20 library, a headless CLI, and acceptance tests. The library owns guest execution and device state. Validated Module objects are immutable and shareable. Each Vm owns its frames, operand stack, globals and bus. A caller must give any one VM exclusive access while it executes or its hardware is being updated.

## Build and run

```sh
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

The default unit tests require only a C++ compiler. Configure with `-DROBOT_TEST_C_GUEST=ON` to build and test the C example using Clang with the wasm32 backend and wasm-ld. Use `-DROBOT_CLANG=/path/to/clang` to select a particular toolchain.

```sh
cmake -DCLANG=clang -DSOURCE=examples/controller.c -DOUTPUT=build/controller.wasm -P sdk/c/build_guest.cmake
./build/robot_sim_cli run build/controller.wasm 3 100 1
```

With Visual Studio, the executable is under `build/Debug/`. The run command takes up to three positive positional limits for ticks, active CU per tick and RAM pages. The example uses one RAM page, completes on tick 2, and reports status 2 and response 143.

## Implementation stages

| Milestone | Files | Acceptance |
|---|---|---|
| M0 | CMakeLists.txt, src/main.cpp, docs, Linux CI workflow | CMake, CTest and CLI help |
| M1 | include/robot_sim/bus.hpp, src/bus.cpp | RAM widths, endianness, all-or-nothing writes, growth, faults, wiring and completion |
| M2 | include/robot_sim/vm.hpp, src/vm.cpp, tests/core.cpp | Decode and validate fixtures, compute suspension/resume, power-mode writes and sleep |
| M3 | sdk/c, examples/controller.c, tests/c_guest.cmake | Compile C and execute the resulting WASM through the CLI |

Multi-VM scheduling, RTC alarm queues, world snapshots, real protocol stacks, structured CLI scenarios and replay are later stages. The test peripheral is a deterministic protocol stand-in. See the guest and MMIO ABI documents for the supported subset.


## Game integration for M4 through M6A

Construct `Simulation` with the game's tick rate and worker count. Pass a `RobotConfig` into `add_robot` for each VM. The configuration belongs to that VM for its lifetime.

| Configuration | Unit or behavior |
|---|---|
| Active and LowPower throughput | Integer CU per simulated second. One instruction costs one CU |
| RAM cap | Number of 64 KiB pages. A module may declare a lower maximum |
| Controller capability | Optional abstract protocol controller, logical pins, target and timeout |
| DMA capability | Optional maximum byte count on the controller |

At each call to `step`, the game supplies replies and external IRQs as a `WorldInput` value. The VM workers only execute their assigned interpreters. The coordinator exposes device commands in ascending Robot ID order after all workers finish. A world adapter can use that order to choose which command gets a contested resource and return the outcomes in the next call. Game peripheral behavior and world-resource decisions belong in that adapter. The CLI provides a deterministic test peripheral.

CU from the selected operating point is allocated at tick entry, with an integer remainder carried across ticks. Power duration is sampled at tick entry. An RTC deadline can wake a VM at the ending boundary and it can execute in the next tick. Safe-point inspection includes the timestamp, fault details and interpreter location. A RAM view remains valid until an operation modifies or removes that VM.

```sh
robot_sim_cli test controller.wasm --robot examples/robot.json --scenario examples/controller_scenario.json --trace trace.txt
robot_sim_cli profile controller.wasm --replay trace.txt --workers 4 --json profile.json
```

The CLI accepts version-1 JSON files. The robot file sets capabilities. The scenario file supports tick rate, tick count, worker count, seed, one external IRQ mask per tick, and final assertions. Assertions cover final power, alarm/wake totals, emitted command count, controller status and typed memory faults. The trace records the effective configuration and boundary inputs. See TRACE_FORMAT.md for the layout.

CMake's Clang integration also builds the runtime fixture, which checks initialized data, a page allocation via memory.grow, and copying bytes into that page. The initial guest profile remains a freestanding i32 WASM subset.
