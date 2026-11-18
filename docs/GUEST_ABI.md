# Prototype robot-wasm32-v1

Deploy a WebAssembly binary version 1 exporting `robot_main_v1` with signature `() -> ()`. The export name declares the prototype ABI version. The interpreter supports i32 functions, direct calls, i32 globals and locals, void structured control flow, integer arithmetic/bit operations/comparisons, select, i32 loads and stores through 32 bits, memory.size/grow, and active data segments for memory zero. A module may export its sole memory. Initial and maximum page limits are applied at instantiation, with the smaller explicit module or hardware maximum governing growth.

Instantiation copies active data into committed RAM and starts the entry frame. The C example uses compiler-initialized storage and the compiler's linear-memory stack layout. A hosted C runtime and malloc are deferred. Modules using imports, start functions, tables, non-i32 types, value-producing blocks, SIMD, threads, or other unsupported instructions/sections are rejected.

Each dispatched instruction costs one CU. The VM tick step samples power at entry and carries half-CU rounding across ticks, using 100% in Active and 50% in LowPower. The guest reads mode changes immediately. Sleep suspends after the current instruction and saves its continuation. Budget exhaustion also preserves the exact continuation. A trap terminates execution of that VM instance.

The interpreter extends normal WASM memory bounds to dispatch ordinary load/store instructions to mapped physical MMIO. The same binary may validate in a standard runtime and trap there when it touches MMIO.


M4 through M6A add the game-facing Simulation configuration and scheduler. That path takes separate integer Active and LowPower CU rates from the game and accounts in simulated nanoseconds. The legacy single-VM positional CLI retains its prototype CU-per-tick interface.

The library validator can opt into the i32 sign-extension instructions through `WasmFeatures::sign_extension`. Enabled and disabled validation paths have fixture coverage. The CLI and SDK build profile use the default setting.
