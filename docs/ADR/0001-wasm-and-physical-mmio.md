# ADR 0001 — WASM32 with physical MMIO

Status: provisional for M0–M3.

Use C++20/CMake and immutable decoded WASM32 modules. Validate the initial i32 subset before instantiation and reject unsupported constructs. A small profile permits test coverage of the interpreter and a freestanding C fixture before extending compatibility. The `robot_main_v1` export versions deployment without requiring a custom section generator.

Scalar memory accesses use the VM-owned bus and a wide effective address. The default MMIO base is relocatable at bus construction. Aligned 32-bit prototype controller registers and a two-pin abstract peripheral exercise the boundary between CPU MMIO and tick-delivered I/O. This register map, two-pin model, fixed CLI CU defaults and half-speed LowPower ratio are prototype defaults. Future scheduler configuration will supply hardware operating points and authoritative world time.

The initial implementation keeps guest code immutable and mutable state per VM. The CLI drives one VM and explicitly advances its bus at tick boundaries. The future multi-VM coordinator will own these boundaries and determine ordering across VMs.

