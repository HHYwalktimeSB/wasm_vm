# Robot Simulator — Technical Design & Implementation Specification

> Status: **Implementation draft v0.3**  
> Date: 2026-11-17  
> Target audience: Coding Agent / C++ implementers  
> Goal: implement a standalone, headless robot computation simulator that can later be embedded as a library in an Axmol-based 2D survival/automation game.

## 0. Read this first: decisions, assumptions, and boundaries

### Confirmed product requirements

- The gameplay is a 2D post-apocalyptic survival and robot automation game. **This document covers the simulator only**, not game progression or balancing.
- A robot has a programmable computer with configurable compute throughput, RAM limit, and attached devices.
- Player programs primarily use **C or Python**. The game should offer ready-made runtimes/templates; advanced players may use lightweight or self-made runtimes.
- Performance modeling is **behavioral, budget-based**, not cycle-accurate CPU architecture simulation.
- Preferred guest format: **WASM32**, with a custom interpreter/runtime and possible future JIT. The final implementation must not depend on Axmol.
- C pointers should be able to access **mapped MMIO directly** with normal loads/stores. There is no guest virtual memory, MMU, or address translation.
- Each VM may execute on **at most one host worker at a time**, and in the first version is scheduled on only one worker within each game tick. It may migrate *between* ticks at a safe point.
- The simulator must build as a reusable C++ library **and** as a standalone CLI executable, with the same simulation core and hardware behavior.
- CLI/headless use is a primary supported workflow for external editors, compilers, debugging, deterministic regression tests, and performance profiling.
- Host game/world must not be mutated directly by guest execution threads.
- Guest-visible power management uses **Active**, **LowPower**, and **Sleep** modes. A virtual RTC provides simulated world time and timer alarms; external devices may trigger interrupts/wake events.
- Busy waiting is legal and consumes compute budget. The guest ABI must **not require a simulator-specific `yield()` call** to idle efficiently; sleeping is exposed through hardware/power-management semantics.
- CPU operating modes expose **compute throughput (CU per simulated second)** and power draw, **not** clock-accurate MHz, arbitrary voltage, overclocking or voltage-damage mechanics. CPU-model operating points are immutable to guest programs.
- Peripherals attach to **logical pins and configurable communication controllers**; CPU MMIO maps controllers (GPIO, protocol controllers, RTC, IRQ, power), not necessarily every peripheral directly. Drivers perform device transactions.
- The simulator guarantees supported **controller-mediated protocol transactions** with correct wiring/configuration at **tick granularity**. No voltage waveforms, bit-level protocol timing, or guaranteed software bit-banging support.
- **DMA is planned as a subsequent milestone.** After successful completion, the receive buffer contains valid data; accessing a valid buffer while transfer is in progress is memory-safe but its data values and conflicting-write effects are *unspecified*.
- External IDE/debugging and the game use the same headless core; **debugging always uses the interpreter**, even when a future optimized JIT backend is enabled for normal execution. JIT is explicitly deferred.
- **No sub-tick wakeups in v1.** A sleep request shorter than a world tick resumes at the next eligible tick boundary, never during the current tick.


### Provisional engineering choices made in this spec

These are starting defaults, not gameplay balance commitments. Make them configurable and record deviations in an ADR (Architecture Decision Record):

1. C++20, CMake, Linux x86-64 first; avoid unnecessary platform-specific implementation.
2. WASM32 MVP-style single-memory, single-thread execution initially. Guest threads, atomics, SIMD, memory64, component model, and WASI OS emulation are out of scope.
3. One canonical simulated step/tick and a stable instruction-cost schedule; each guest computer receives integer compute capacity based on elapsed simulated time and its current operating mode. External device changes, interrupt delivery and sleep wakeups occur only at tick boundaries in v1. No sub-tick execution of external events.
4. RAM mapped from physical address `0x00000000` upward; example MMIO window from `0x40000000` upward, primarily for internal controllers. **Addresses are examples and must be centrally configurable**.
5. Guest memory access uses an explicitly defined **custom WASM32 physical-address extension**. Standard third-party WASM runtimes are not expected to execute MMIO-dependent programs correctly.
6. Use immutable per-tick world inputs and per-VM output buffers, then apply commands at a deterministic barrier. Keep RTC and device wakeups deterministic and independent of host wall time.
7. Initial and debugging execution backend is the interpreter; JIT is optional and deferred until correctness and profiling justify it. Debug-enabled VMs must execute in interpreter mode.

### Explicitly *not* decided here

- Which template programs exist; whether templates are unlocked initially or extracted from found robots.
- Number of CPU tiers, gameplay RAM capacities, resource costs, difficulty-specific tuning, and how code is discovered in the world.
- Final Python dialect/interpreter implementation, precise standard-library coverage, and third-party-library support.
- Guest OS, multicore guest CPU, virtual memory, full networking stacks, physics, combat, exploration AI, or UI.

**Do not invent game rules to fill these gaps.** Use placeholders/configuration examples and list unresolved decisions.

---

## 1. Architectural overview

```text
                           +----------------------+
  CLI / Test Harness -----> |                      | <----- Axmol Adapter (later)
                            |  Simulation Library  |
  Scenario / Trace Files -->|                      |-----> Logs / Stats / Traces
                            +----------+-----------+
                                       |
                            +----------v-----------+
                            | Tick Coordinator      |
                            | scheduler / barriers  |
                            +----------+-----------+
                                       |
                             +---------v----------+
                             | Worker Pool         |
                             | VM instance tasks   |
                             +---------+----------+
                                       |
                       +---------------v----------------+
                       | VM Instance (one guest robot)   |
                       | execution state, stack, RAM     |
                       | MMIO bus, budget, event buffers |
                       +---------------+----------------+
                                       |
                       +---------------v----------------+
                       | Validated WASM Module /         |
                       | optional shared compiled code  |
                       +--------------------------------+
```

**Ownership:** Simulation library owns VM execution state and virtual devices. World Host owns authoritative physical/world state, positions, collisions, inventory, resource transfers and sensor truth. Axmol owns rendering and input presentation. Only adapters depend on Axmol.

### Repo structure (proposed)

```text
robot-simulator/
  CMakeLists.txt
  cmake/
  docs/
    DESIGN.md
    MMIO_ABI.md
    GUEST_ABI.md
    TRACE_FORMAT.md
    ADR/
  include/robot_sim/
    Simulation.hpp
    Types.hpp
    Config.hpp
    Snapshot.hpp
  src/
    wasm/       # loading, validation, decoding, interpreter
    vm/         # instance, execution state, memory, traps
    devices/    # MMIO controllers, logical pins, peripherals, RTC/IRQ, later DMA
    power/      # CPU power states, CU throughput and power policy
    scheduler/  # tick coordinator and worker pool
    host/       # host-service interface and event buffers
    trace/      # recording and replay
  cli/
  sdk/c/       # guest headers, linker script/build profile
  sdk/python/  # future language bindings / runtime package
  examples/
  tests/unit/
  tests/integration/
  tests/conformance/
  tests/scenarios/
  benchmarks/
```

`robot_sim_core` should have **no Axmol dependency**. `robot_sim_cli` links it. Future `robot_sim_axmol_adapter` links it without duplicating simulator logic.

---

## 2. Guest execution format and compatibility

### 2.1 Why WASM32

Reuse existing C compiler infrastructure and a well-defined stack-machine instruction format rather than implementing a proprietary C compiler backend. A game-provided build wrapper should hide target flags from normal players. Advanced players can supply their own modules that match the declared ABI.

**Important:** WASM's normal linear-memory semantics do *not* provide MMIO; this VM intentionally extends the semantics of memory instructions for mapped device addresses. The compatibility and security consequences must be documented prominently.

### 2.2 Guest compatibility profile: `robot-wasm32-v1`

- Only validated `.wasm` modules; reject malformed or unsupported features before executing anything.
- MVP baseline first: integer and floating-point arithmetic, locals/globals, structured control flow, direct/indirect function calls as required by real C toolchain outputs, supported load/store sizes and extensions, single 32-bit-addressed memory, initialization/data sections, and basic imports for explicitly allowed host functions.
- **Feature-gate** optional features generated by toolchains (`bulk-memory`, `sign-extension`, etc.). Do not claim a feature is supported until an integration fixture exercises it.
- No multi-memory, memory64, WASM threads, shared memory, WASI filesystem or sockets, reference-type extensions beyond what is strictly needed.
- A module's requested initial/max memory and import set must be validated against its robot hardware configuration and policy.
- Unknown opcodes, undeclared imports, and unsupported module features produce a structured `LoadError`, never silent misexecution.

### 2.3 Execution lifecycle

1. Load bytes and parse/validate module.
2. Resolve configured imports against a **versioned, minimal Robot Host ABI**.
3. Instantiate VM, allocate initial RAM, apply data initializers, initialize guest globals/table.
4. Run start function (if declared), under explicit compute budget and the same safety rules.
5. Enter the configured guest entrypoint, then execute until (a) budget exhaustion, (b) guest power/sleep transition, (c) an explicit asynchronous host wait, (d) normal exit, or (e) trap.
6. Preserve operand stack, locals, call frames, instruction cursor and memory across ticks.

A guest program must be pausable *within* a long pure-computation loop without requiring a host API call. `BudgetExhausted` is **normal suspension**, not a fatal WASM trap. A guest `unreachable`, invalid memory access, invalid division, etc. produces a **fault** with diagnostic context.

---

## 3. Physical memory and MMIO — normative semantics

### 3.1 One physical address map, no virtual addressing

For guest C code, a `uint32_t` pointer value denotes a **guest physical address**. The VM classifies each effective load/store by that address. No paging, translation tables, ASIDs, TLB, or guest process isolation within the same computer. Host process memory is **never** directly exposed to guest pointers.

Example configuration (not fixed balance values):

| Guest address range | Type | Meaning |
|---|---|---|
| `0x00000000..RAM_LIMIT-1` | RAM | dynamically committed guest linear memory |
| `RAM_LIMIT..0x3FFFFFFF` | Unmapped | memory fault |
| `0x40000000..0x400000FF` | MMIO | example GPIO/pin multiplexer controller |
| `0x40000100..0x400001FF` | MMIO | example SPI/I²C/UART controller |
| other addresses | Unmapped/reserved | memory fault |

`RAM_LIMIT` is a runtime parameter bounded by robot hardware and WASM32 representation. **Allocated/committed memory** may be smaller; a load/store to a reserved but not yet committed RAM page faults until `memory.grow` succeeds.

### 3.2 Load/store dispatch

- Compute WASM effective address using a sufficiently wide integer (`uint64_t`), including immediate offset, to detect wrap/overflow **before** truncation.
- If **all bytes** of the access are within currently committed RAM, perform little-endian RAM read/write.
- Otherwise, if **all bytes** are within one mapped MMIO region and the access width is permitted for that register, dispatch to `Device::read/write` with region-relative offset.
- Otherwise trap `UnmappedMemory` / `MisalignedMMIO` / `InvalidRegisterAccess` / `CrossRegionAccess`, as appropriate.
- Access spanning RAM/MMIO or two different devices **must never be split implicitly**.
- RAM follows WASM's allowed unaligned-access behavior; MMIO may require alignment and exact widths, documented per register.
- `volatile` in guest C prevents certain compiler optimizations but does **not** implement MMIO by itself; the VM bus does.
- Guest memory never references host process pointers.

```cpp
Expected<uint64_t, MemoryFault> MemoryBus::load(
    uint64_t effectiveAddress, AccessWidth width);
Expected<void, MemoryFault> MemoryBus::store(
    uint64_t effectiveAddress, AccessWidth width, uint64_t value);
```

### 3.3 Bulk operations and memory size

Define carefully before implementing toolchain-generated bulk memory instructions:

- `memory.size`: committed **RAM** size in 64 KiB pages, not max physical address and not MMIO range.
- `memory.grow`: increases committed RAM pages only up to configured limit, returning prior page count or the standard failure value; never maps into MMIO.
- Memory initialization, `memory.copy`, `memory.fill`: **RAM only** in v1; if a source/destination span would touch MMIO or unmapped memory, fault without partial modifications.
- Heap/stack allocation is the guest runtime's responsibility. The VM enforces bounds and maintains counters, but does not impose one mandatory allocator.
- The entire guest pointer model is defined only for modules targeting `robot-wasm32-v1`; standard WASM modules using only RAM can remain compatible with normal semantics.

### 3.4 Example guest access pattern

The CPU directly accesses **controller registers**, not arbitrary external device registers. C pointers still provide direct MMIO access to controller addresses:

```c
#include <stdint.h>
#define REG32(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define SPI0_CTRL   REG32(0x40000100u)
#define SPI0_TXDATA REG32(0x40000104u)
#define SPI0_STATUS REG32(0x40000108u)

void send_sensor_command(uint32_t command) {
    SPI0_CTRL = 1u;       // illustrative only: configure/enable controller
    SPI0_TXDATA = command; // enqueue/submit controller transaction
}
```

This must produce ordinary WASM loads/stores reaching the controller through the game VM `MemoryBus`. The specific register layout is **TBD and must be versioned in `MMIO_ABI.md`**. The same WASM module need not work in an unmodified standard WASM runtime.

### 3.5 Device and ownership model

Separate three concepts:

- `IMemoryMappedController`: internal CPU-visible registers, reset/configuration, status/IRQ; immediately updated local registers where documented.
- `IConnectionGraph` / `PinMux`: logical pin endpoints, pin function selection, controller-to-device wiring, compatible port/protocol relations; no analogue voltage/waveform simulation.
- `IPeripheralDevice`: externally attached device with its own state and protocol transaction behavior, updated at defined tick boundaries.

Example interfaces (non-final):

```cpp
class IMemoryMappedController {
public:
    virtual ~IMemoryMappedController() = default;
    virtual DeviceAccessResult read(uint32_t offset, AccessWidth width) = 0;
    virtual DeviceAccessResult write(uint32_t offset, AccessWidth width,
                                     uint64_t value) = 0;
    virtual void publishTransactions(TransactionSink& out) = 0;
};
class IPeripheralDevice {
public:
    virtual ~IPeripheralDevice() = default;
    virtual void applyInputs(const PeripheralInputs&) = 0;
    virtual void process(const ProtocolTransaction&, TransactionResultSink&) = 0;
    virtual void advanceTick(const TickContext&) = 0;
};
```

A controller owned exclusively by one VM may make its **local** status/configuration writes visible to that VM immediately. Any shared peripheral, world state or cross-VM communication must be handled through deterministic tick-stage queues; worker threads must not directly mutate Axmol objects or race on shared device RAM.

Initial devices/controllers: RTC, interrupt controller, CPU power controller, and **one simple abstract protocol controller with a connected test peripheral**. GPIO and specific SPI/I²C/UART semantics may follow incrementally. Do not require a guest-visible `yield` register.

### 3.6 Pin connections and protocol transaction semantics — normative

1. The game/hardware description declares logical pins, alternate functions, external endpoints, supported controllers and device capabilities. A connection is meaningful only if the pin mapping and controller configuration agree.
2. CPU code communicates through **MMIO controller registers** and device driver routines; peripherals are generally not independently memory-mapped. A later CPU-integrated peripheral *may* be directly MMIO mapped if explicitly defined.
3. The simulator verifies abstract requirements for supported transactions: connectivity, compatible protocol, controller enabled/configured, device address/chip select as applicable, powered device, and bus ownership/busy rules.
4. **Guarantee:** if supported wiring and settings are valid and the peripheral is available, a supported transaction eventually completes per documented tick-boundary timing and returns the defined result. Normal device errors must be reported in controlled status/timeout semantics, not host crashes.
5. Peripheral state and transactions are advanced **only at fixed world tick boundaries**; no sub-tick pin toggles, bit timing, clock edges, electrical propagation, signal integrity, analogue voltage, or guaranteed GPIO bit-banging. Controller-register reads and writes during guest execution may have immediate **controller-local** effects, not immediate external device effects.
6. Communication has an explicit state machine: `Idle -> Queued/Busy -> Completed | Failed | TimedOut`. Completions, error status and IRQ become guest-visible together at the documented next boundary. Multiple contenders are resolved in stable order independent of host worker scheduling.
7. Incorrect pin mapping/configuration produces explicit controller failure, NACK/timeout/no-response as appropriate to its protocol; it never creates a random valid response. Unsupported protocols must fail with a documented error, not pretend to communicate.
8. Every controller profile specifies request queuing, buffer sizes, bandwidth/latency *in game ticks* if relevant, addressing, error responses, and interrupt behavior. Balancing values remain in configuration, not in the interpreter.
9. The same profile/configuration and scenario must produce matching behavior in CLI, tests and the Axmol adapter.

### 3.7 DMA and communication buffer visibility — normative; DMA implementation deferred

DMA is an optional physical-device capability, **not** a Guest VM opcode, and does not consume Guest CPU CU while moving data; its setup/driver code consumes CU as usual. The DMA controller uses the same checked Guest RAM/bus abstraction as the CPU, without obtaining host pointers from guest addresses.

The public programming contract is intentionally limited:

- **Receive:** after the corresponding transfer reports **successful completion**, the destination buffer contains the full defined received payload, provided guest software did not perform conflicting writes while it was busy.
- **Transmit:** source buffer contents must be prepared before launch; any guest write during active transfer makes the sent data/result **unspecified**. The official driver must treat source/destination buffers as unavailable for conflicting use until completion.
- **While Busy:** CPU reads/writes to an otherwise valid mapped RAM buffer are allowed without a fault *solely because the transfer is active*, but intermediate byte values, observation order, partial transfer visibility and effects of conflicting writes are **unspecified**. This does not allow out-of-bounds/unmapped access or bypass MMIO restrictions.
- **After failure/timeout/cancel:** payload is not guaranteed valid; status code is guaranteed and caller must decide how to recover.
- **Ordering:** commit promised receive data **before** publishing `Completed` status or interrupt. Publish completion and data at a defined tick boundary (v1 implementation may use atomic-at-completion copy). During transfer do not promise streaming/partial progress visibility.
- **Determinism:** unspecified Guest-visible intermediate values may change across implementations, but within a fixed simulator version, seed and inputs, results remain reproducible. Host thread races and out-of-bounds access are forbidden regardless of the Guest contract.
- **Concurrency:** DMA may advance while the CPU is in Sleep, but no background worker may unsafely mutate shared RAM during concurrent VM execution. Queue/copy or serialize writes at the tick barrier. Avoid assuming a cache-coherence system: CPU cache simulation is out of scope.
- **Addressing:** v1 DMA may support peripheral<->RAM only; RAM<->RAM, scatter/gather, circular DMA, descriptor rings, bandwidth contention and partial-progress observability are deferred. A future DMA engine must respect the unchanged completion contract.

A conforming official driver waits for success before exposing receive data to higher-level code, and does not modify an active transfer's source/destination buffer. Test harnesses should deliberately access active valid buffers to prove **memory safety**, but must not assert any particular intermediate values.

---

## 4. Simulation time, CPU power management and event-driven scheduling — normative

### 4.1 Distinguish the three clocks

1. **Host wall clock:** elapsed real time, used by the Axmol adapter to request simulation progress; never directly exposed by guest RTC reads.
2. **Simulation/world clock:** authoritative monotonic integer simulated timestamp (recommended microseconds or finer integer units). It advances only for completed fixed world ticks. The game may target ~30 real minutes per in-game day during normal play, and 1:1 real/world time in a development session; both policies belong to the **game front end**, not to this simulator library. If the host cannot keep up, the simulated day becomes longer in wall time rather than skipping simulation ticks.
3. **Guest CPU throughput:** configurable *compute units per simulated second* and the associated power-state multiplier; controls how many instructions the VM may execute, not the RTC rate.

The CLI drives the same world clock with explicit `step(n)` calls and may run as fast as the host allows. CLI/headless tests **must not** depend on real-time sleep to move the RTC forward. `tickRate` is an integer or rational simulation setting; use fixed-point/integer accumulation to avoid drift.

### 4.2 CPU power-state machine

| Guest-visible state | CPU execution | Simulated compute allowance | Simulated energy | Host execution strategy |
|---|---|---|---|---|
| `Active` | executes normally | nominal configured rate | configurable active draw | schedule runnable VM |
| `LowPower` | **continues executing** | reduced configured rate | configurable lower draw | schedule smaller budget |
| `Sleep` | executes **no ordinary instructions** | zero while asleep | configurable sleep/standby draw | remove VM from runnable queue until a wake condition |

- CPU modes specify **CU per simulated second** and **power draw**, not a physical CPU clock frequency. CPU-model performance/power operating points are developer-defined and immutable from Guest code; Guest can only select supported modes. No voltage register, arbitrary frequency, overclocking or overvoltage/thermal-damage mechanics. `LowPower` is not `Sleep`: polling still consumes CU and host work.
- The VM records a guest-visible requested state and a separate internal scheduler status (`Ready`, `Running`, `SuspendedBudget`, `WaitingForHostIO`, `PausedByDebugger`, `Faulted`, `Exited`). A debugging pause or pending AI request is **not** a guest CPU sleep and must not silently change the guest power registers.
- `Active`/`LowPower` transitions take effect at a documented instruction boundary (v1: immediately after the validated power-register write or power intrinsic completes). The executor must stop/recalculate the current per-tick allowance before executing another guest instruction; switching state cannot grant a second full tick budget.
- Entering `Sleep` is a controlled **non-fault suspension** after the triggering guest instruction. Preserve the next instruction cursor, call frames, globals, RAM and all device state; resume at the documented next instruction upon wake. The guest is not killed and does not execute a busy loop while sleeping.
- Busy waiting is intentionally supported and consumes compute budget. **Do not auto-recognize/skip polling loops**, as this would change guest-visible counters, timeouts and power cost. The interpreter is still preemptible on budget exhaustion even in `while(1) {}`.
- A built-in SDK may provide `robot_sleep_until`, `robot_sleep_for` and `cpu_wait_for_interrupt` in terms of RTC, interrupt registers and power state; all must be **optional** open-source wrappers. Do not require or expose a mandatory guest `robot_yield()` mechanism. An internal VM scheduler suspension facility is allowed.

### 4.3 Budget calculation and accounting

For active states let `nominalComputeUnitsPerSecond` be the robot hardware compute rate and `modeRatio` the configured rational **throughput** multiplier (`Active` = 1, `LowPower` < 1). For one completed world tick, calculate the allowance using integer/fixed-point arithmetic and retained fractional remainder. Initially, for a tick that stays in one mode:

```
allowance_n = floor(remainder + nominalRate * modeNumerator
                    / (tickRate * modeDenominator))
remainder   = fractional part carried to subsequent ticks
```

Implement the arithmetic without overflow or floating-point nondeterminism; exact representation may use numerator/denominator and wide integers. A mode change during Guest execution is observable immediately for CPU state/power registers, but **v1 computation-credit rate changes take effect from the following tick**. Sleep immediately stops ordinary instruction execution, regardless of remaining credits. There is no sub-tick timing model and no mid-tick new CU credit issuance; changing modes can never mint a second budget. Account the power-state change using a documented tick-granularity convention rather than fictional instruction-level elapsed nanoseconds. Tests must cover rapid toggling. A future version may integrate CU-based virtual CPU elapsed time, but that is not required for v1.

- Version a stable `CostTable`: default one cost unit per guest instruction, optionally extra fixed cost for MMIO/host calls. Interpreter and future JIT must charge equivalent **logical guest cost**, independent of host machine speed.
- `SuspendedBudget` is normal preemption, not a fatal WASM trap. On next tick, a runnable VM obtains its new allowance and resumes at the exact instruction boundary.
- **Unused ordinary compute credits do not accumulate without limit** while in `Sleep`, waiting for external I/O or debugger-paused. Cap or discard them by explicit policy; v1 discards unused credits at each tick boundary, preserving only fractional rounding remainder. Wakeups must not produce a burst of accumulated CPU speed.
- Energy tracking uses **simulated tick duration × selected CPU-model power point** (plus optional device power); draw may include separate active/idle/standby values. The operating-point table is immutable to Guest code but developer-configurable by CPU model. Energy accounting is independent of host speed and does not implement voltage/frequency transients. Sleep and RTC/peripherals can still consume standby power. Exact watt values remain gameplay-configurable.

### 4.4 RTC device and timebase

Provide a guest-readable RTC that follows the *authoritative simulation/world clock*, never host elapsed wall time, host worker speed, or guest CPU mode.

Minimum abstract register behavior (allocate concrete physical offsets in `MMIO_ABI.md`):

- `RTC_TIME_LO` / `RTC_TIME_HI`: 64-bit monotonically increasing simulation timestamp, read with a **documented coherent-snapshot/latching rule** to avoid torn two-register reads.
- `RTC_ALARM_LO` / `RTC_ALARM_HI`: a 64-bit absolute deadline in that same timebase; define an atomic latch/commit register or equivalent protocol for updating the 64-bit deadline.
- `RTC_CONTROL`: alarm enable and optionally periodic behavior (periodic alarms deferred in v1).
- `RTC_STATUS`: pending/expired and acknowledge semantics. An already expired alarm must have defined immediate-pending behavior at the next event boundary.

RTC runs while the guest CPU is `Active`, `LowPower` or `Sleep`; it is advanced by simulation ticks. Configuration and guest visibility must be deterministic under 1/N worker counts and CLI replay. **Do not assume each VM independently increments a clock from executed instructions.**

### 4.5 Interrupt controller and wake semantics

The interrupt controller exposes latched **pending** bits, enable/wake masks and acknowledgement behavior for RTC and device sources. Specify all accesses and bit positions in `MMIO_ABI.md`.

- Device events raise a pending interrupt even if the CPU is sleeping. Pending bits remain latched until explicitly acknowledged/cleared according to the source contract.
- Distinguish **wake eligibility** (an enabled pending wake source can exit `Sleep`) from **interrupt-handler dispatch** (if/when guest interrupt delivery is implemented). V1 may support *wake-and-resume without a guest ISR*; a full guest ISR/interrupt ABI is a separate milestone.
- A guest sleep entry is an **atomic decision at a VM safe point**: after processing the power command, check latched eligible pending events. If one is already present, do not become indefinitely asleep. This avoids the common check-then-sleep lost wakeup.
- Multiple simultaneous sources resolve by a stable documented priority/order. Events coming from other VMs/world must be buffered and delivered at deterministic boundaries, not according to host thread timing.
- With v1 **tick-boundary wake granularity**, a sleep request of 1 ms when a world tick represents more than 1 ms **cannot wake within that same tick**. Wake at the earliest *subsequent* tick boundary at or after the target deadline. Even when the requested deadline is already due, no re-entry within the current tick after entering Sleep. Do **not** claim sub-tick response latency. The event queue should retain timestamps so finer-grained scheduling can be added later.
- Waking does **not** erase an interrupt pending bit by itself. On wake, the power controller returns to a documented mode (v1: `Active`, unless the guest contract configures otherwise).

### 4.6 Host-side scheduling without guest `yield`

Maintain per-VM scheduler metadata and event registrations outside guest memory. A sleeping VM is **not** scheduled to execute on every tick merely to poll an MMIO flag. RTC alarms use a deadline priority queue; device wakeups enqueue the affected VM on event arrival. Avoid a full sweep of sleepers per tick in the steady state. Independently advance time/energy accounting and device state through the clock/event system.

- At each fixed world tick, advance timestamp/RTC and determine due wakes using a **single documented ordering** relative to the world snapshot and device input phase.
- Host async I/O (AI/network) is not guest CPU Sleep by default: requests complete on a separate queue and are delivered deterministically at subsequent tick boundaries. The VM's `WaitingForHostIO` state is independent of hardware Sleep.
- No worker waits synchronously for RTC alarms, network responses, or wall-clock timers.
- A sleeping VM still owns its RAM, module state, device state and pending interrupts; it may migrate to another worker only at a safe point after wake.

### 4.7 Required minimal guest-facing example (conceptual)

```c
// Register offsets and bit definitions come from the versioned SDK header.
uint64_t deadline = rtc_read_time() + RTC_SECONDS(5);
rtc_set_alarm(deadline);
irq_enable(IRQ_RTC);
cpu_enter_sleep(); // preserves VM state, stops ordinary guest execution
// After wake: read/ack pending status according to documented ABI.
```

The C helper functions above are source-level convenience wrappers over MMIO, **not magic VM `yield` calls**. `cpu_enter_sleep()` may use a power MMIO register or a documented special host intrinsic; if implemented as a write, the VM must suspend at the instruction boundary immediately after that write. The spec does not yet assign final register addresses or a full ISR ABI.

---

## 5. Concurrency and determinism

### 5.1 VM ownership invariant

- A VM has exactly one active executor at a time.
- During one tick it is scheduled on a single worker; no intra-VM parallelism in v1.
- At the barrier, the VM is quiescent and may be reassigned for next tick.
- Do not allow inspector, CLI debugger, game frontend, or another worker to mutate VM memory while executing. Use command queues and copies/read-only snapshots.
- Module code/validated immutable metadata may be shared across VM instances. VM stacks, RAM, mutable globals, tables, devices and budget are separate.

### 5.2 Tick phases

1. **Input/clock:** advance authoritative simulation timestamp for this fixed tick; freeze world snapshot, deliver due RTC/device wake events (with defined boundary order), sensor readings and packets; commit queued safe-point debug/config changes.
2. **Execute:** schedule only runnable independent VM tasks onto a fixed worker pool; each executes its allocated tick CU (mode changes affect the next tick’s credit rate in v1); sleeping VMs do not run ordinary guest instructions. Controller-local MMIO writes are ordered in the VM, external protocol transactions queued.
3. **Collect:** workers publish per-VM command/event buffers, stats and faults.
4. **Commit:** deterministic stable order (e.g. `(tick, robotId, localSequence)`); resolve logical-pin/controller transactions, validate and apply World Host commands, advance peripherals at this tick boundary, commit scheduled DMA transfers and enqueue resulting statuses/IRQs for the next defined input boundary. Wake delivery uses the documented following boundary.
5. **Publish:** expose immutable simulation snapshot for frontend and next tick inputs.

If a host command has non-commutative effects (two robots compete for one item), World Host resolves it deterministically. Guest observations of changes to shared external state are not immediate inside the tick. MMIO devices can update their **private local register state** within the tick; external world effects are buffered.

### 5.3 Worker pool

- `workerCount=1` and `workerCount=N` must produce the same **guest-observable** results for a fixed seed/input/VM execution policy.
- No one-thread-per-robot architecture.
- Avoid shared writes from workers except bounded thread-safe result collection or disjoint slots.
- Initial scheduler: stable deterministic VM order with jobs assigned to workers; work stealing may follow only after tests prove determinism.
- Use a barrier per tick. Prefer simple locks/condition variables to premature lock-free complexity.
- Expose `maxGuestInstructionsPerHostStep` or equivalent safety guard for CLI tests to prevent runaway work configurations.

---

## 6. Host integration API (library)

The following is a **proposed interface**, to refine while implementing. Avoid exposing interpreter internals publicly.

```cpp
using RobotId = uint64_t;

struct SimulatorConfig {
    uint32_t ticksPerSecond = 30;
    uint32_t workerCount = 1;
    uint64_t randomSeed = 0;
};

struct RobotConfig {
    uint64_t computeUnitsPerSecond = 100000;
    // immutable CPU model: fixed CU throughput/power operating points; RTC/IRQ config
    uint32_t maxRamBytes = 1024 * 1024;
    // module compatibility profile, RAM map, controllers, pin topology, peripherals
};

class Simulation {
public:
    explicit Simulation(const SimulatorConfig&);
    RobotId addRobot(const RobotConfig&, std::span<const std::byte> wasm);
    void removeRobot(RobotId);            // safe-point operation
    void setInputs(const WorldInput&);    // inputs for next step
    StepResult step(uint32_t ticks = 1);  // synchronous v1, internally parallel
    RobotSnapshot inspect(RobotId) const; // snapshot only at safe point
    Statistics stats() const;
};
```

`StepResult` contains ordered commands, faults, logs and snapshot identifiers. Provide structured errors instead of throwing across external C ABI boundaries. Consider an optional stable `extern "C"` wrapper for engines/other language integration after the C++ API is validated.

The host adapter for the Axmol game must only queue commands and read published snapshots; no direct Sprite/Scene access from VM workers.

---

## 7. Headless CLI — first-class deliverable

Executable name: `robot-sim` (provisional).

### Required initial commands

```sh
robot-sim validate --program build/miner.wasm
robot-sim run --program build/miner.wasm --robot configs/miner.json \
  --scenario scenarios/test_mine.json --ticks 1000 --seed 1 --workers 1
robot-sim profile --program build/miner.wasm --robot configs/miner.json \
  --scenario scenarios/test_mine.json --ticks 10000 --json out/profile.json
robot-sim test tests/scenarios/
```

`run` must support `--workers N`, `--trace path`, `--json path`, reproducible `--seed`, and nonzero exit codes for invalid module, trap, scenario failure, and CLI misuse. The CLI must operate without display or Axmol installation.

### Required data formats

Use versioned JSON schema for configuration and human-editable scenarios. At minimum:

- `SimulatorConfig`: tick rate, worker count, random seed.
- `RobotConfig`: CPU model + immutable operating points, RAM cap, program path, internal controller MMIO mappings, logical pins/connections, external peripheral profiles, RTC/IRQ setup and optional DMA capabilities.
- `Scenario`: initial peripheral/world input, pin connections and scripted changes, protocol traffic, power/IRQ events, optional DMA completion expectations, emitted commands/assertions.
- `Statistics`: ticks, consumed compute units, guest instructions, RAM committed/peak, controller MMIO accesses, protocol transactions/completions/errors, optional DMA bytes/completions, traps, per-power-state simulated duration, sleeps, wakes, RTC alarms and guest idle time.

**Do not report native host CPU instruction counts as guest compute usage.**

### Console diagnostics

- Robot ID, VM state, fault type, WASM module function/instruction location, effective guest address (for memory faults), tick, budget and device.
- Optional per-tick event and MMIO trace, with filters to avoid huge logs.
- Stable machine-readable JSON output as well as readable console summaries.
- CLI program arguments/settings must not provide arbitrary access to host filesystem/network from guest code.

---

## 8. Debugging and replay

### MVP debugger (CLI)

1. Inspect VM state at tick boundaries: internal scheduler status **and distinct guest CPU power mode**, compute budget, simulated RTC time, pending/wake-enabled IRQ masks, memory committed, execution location, basic stack summary and device registers.
2. Trace faults and MMIO operations with guest addresses.
3. Pause execution after N ticks or on error, export a concise state report.
4. Optional instruction stepping/breakpoints **after** the basic interpreter is stable.

### External tools

- **Debug mode forces the interpreter backend**, even when JIT is enabled for ordinary runs. Debugging must not silently run optimized machine code. In v1, attach/restart the VM in interpreter mode; hot switching from a running JIT frame is not required.
- Future VS Code integration uses a game-authorized local development session/IPC to enable the game frontend’s 1:1 world-time mode. The VM/CLI remains usable independently and never infers game speed from editor focus.

- Provide a C guest header `robot_mmio.h`, example linker/build config, and a `robot-build` wrapper or CMake toolchain file. Normal players should not need to understand compilation target flags.
- Support independent C code unit tests on host through a **mock SDK backend**; simulated hardware behavior must be consistent with the actual devices wherever the same behavior is modeled.
- For Python, define the guest API contract early, but defer the actual interpreter choice until a minimal C/WASM proof of concept works.
- DWARF/source-level debugging and VS Code DAP integration are **future milestones**; do not promise them in v1. Debug info alone does not make a custom WASM VM GDB-compatible.

### Deterministic trace and replay (after core execution)

Record format must include simulator version, module hash, cost-table version, scenario/config, seed, all nondeterministic external inputs, tick/time of delivery, and host responses (including AI responses). Replay injects recorded inputs without sending external requests. Define checksums of guest-visible state to validate repeatability.

---

## 9. Safety, limits, and correctness

- Treat guest WASM and user-provided artifacts as untrusted.
- Validate module sections, integer lengths, indices, import signatures, stack typing and feature flags. Prefer reusing a mature validator if possible; **do not execute unchecked binaries**.
- Ensure all guest reads/writes use checked addressing; never cast guest addresses to host pointers.
- Bound guest RAM, module bytes, table sizes, function nesting/call frames, execution-stack size, number of VMs, emitted events and pending host requests.
- Bound trace/log size and enforce configured policies for guest requests.
- Separate guest instructions’ normal budget exhaustion from genuine traps and from VM implementation defects.
- Restrict guest host calls to an explicit allowlist. No arbitrary filesystem, shell or networking capabilities are provided by default.
- Implement deterministic fault handling for unmapped, out-of-bounds, cross-region, misaligned MMIO and invalid device register access.
- Add fuzz/property tests for WASM binary parsing, address ranges and instruction decoding before allowing untrusted community modules.
- Version the guest ABI, MMIO register layout, power-state/RTC/interrupt semantics, cost table and trace format; reject incompatible versions with useful errors.
- Ensure exceptions/faults cannot propagate into Axmol rendering threads or crash the host application.

---

## 10. Development plan for Coding Agent

### Milestone 0 — repository skeleton and architecture contract

**Deliverables:** CMake library + CLI + tests; `DESIGN.md`, `MMIO_ABI.md`, `GUEST_ABI.md`, ADR outlining guest format and memory extension; CI build on Linux.

**Accept:** `cmake`, build and CTest succeed; CLI `--help` works; core has zero Axmol includes or dependencies.

### Milestone 1 — memory bus and virtual devices (no WASM yet)

**Deliverables:** checked little-endian RAM; commit/grow behavior; controller mapping table; simple GPIO/abstract protocol controller register stubs and a connected test peripheral; RTC/power/IRQ register interfaces (initially stubbed if necessary); typed memory faults; pin topology schema; unit tests.

**Accept:** correct 8/16/32/64-bit RAM accesses where applicable; controller MMIO read/write with width/alignment rules; valid/invalid logical pin connection tests; no partial writes across regions; high-address accesses never allocate enormous RAM; memory size/grow follow configured caps.

### Milestone 2 — minimal validated WASM interpreter

**Deliverables:** parse/validate a defined feature subset; stack/call frames; instruction dispatch, budget suspension, immediate power-register effects and sleep suspension; guest module lifecycle.

**Accept:** a hand-authored/compiled WASM fixture runs; an infinite loop suspends on budget exhaustion; Active/LowPower transitions take effect deterministically; Sleep preserves the next instruction; state resumes exactly; unsupported features rejected with useful errors; interpreter and test harness agree on simple arithmetic/control-flow cases.

### Milestone 3 — real C toolchain + pointer-to-MMIO end to end

**Deliverables:** small guest SDK header + reproducible Clang build script; C controller/peripheral example; data initialization, memory allocation and control flow support needed by example; explicit guest ABI version.

**Accept:** `*(volatile uint32_t*)0x40000104 = COMMAND` (inside a correctly built guest module) writes the communication controller register; subsequent correctly wired/configured transfer reaches the external test peripheral at a tick boundary; same C module works under CLI with RAM bounds; C compilation does not require the developer to hand-author WASM.

### Milestone 4 — simulation ticks and multi-VM deterministic scheduling

**Deliverables:** `Simulation` library API, fixed-tick loop, 1-worker/N-worker execution, immutable world snapshot, RTC alarm priority queue, interrupt pending/wake rules, runnable/sleeping queues, controller transaction processing, peripheral tick updates and stable command commit.

**Accept:** 100 independent VM instances with isolated RAM; sleeping VMs consume no guest instruction budget or worker execution slots; a sub-tick-duration sleep wakes no earlier than the next tick; RTC alarm and device IRQ wakes are deterministic; valid controller/connection transactions finish as specified; no concurrent execution of the same VM; identical input produces identical observable results on 1/2/4 workers; two robots competing for one resource resolved predictably.

### Milestone 4A — supported external protocol contract (after scheduler)

**Deliverables:** one end-to-end protocol controller/driver, logical-pin validation, deterministic queued transaction results, error/timeouts, completion IRQ, and proper support for peripherals updated only at tick boundaries. Start with an intentionally small abstract protocol, then add real SPI/I²C/UART controllers in individually reviewed milestones.

**Accept:** correct config delivers data; wrong pin/function/address produces documented failure; no sub-tick external response; repeated 1/2/4-worker runs match.

### Milestone 5 — CLI tests, profiles, fault diagnostics

**Deliverables:** `validate`, `run`, `profile`, `test`, JSON outputs, example scenarios, power-state/RTC/interrupt stats and regression tests.

**Accept:** scripted simulator test can assert commands, power transitions, alarm wakes and trap type; profile reliably reports instruction counts, compute consumption, MMIO accesses, sleeping durations and peak RAM; no graphics library required.

### Milestone 6 — runtime compatibility and replay

**Deliverables:** richer C runtime fixtures (`malloc`, startup, `memcpy`, `memory.grow` as needed), explicit optional WASM extensions, replay trace schema and playback, debug inspect at safe points.

**Accept:** reproducible record/replay with trace checksum; automated tests against C fixtures; documented supported/unsupported features; compatibility regressions detected.

### Milestone 6A — optional DMA controller (after protocol communication)

**Deliverables:** checked peripheral-to-RAM and RAM-to-peripheral DMA at completion boundaries, status/IRQ behavior, buffer ownership contract and safe concurrent handling. No streaming-buffer visibility, cache-coherence or cycle-accurate arbitration.

**Accept:** completed successful receive produces complete valid data before completion IRQ; Guest accesses to valid active buffers do not crash; tests intentionally **do not assert intermediate buffer values**; out-of-bounds accesses fault as usual; deterministic replay and multi-worker execution match.

### Milestone 7 — Python runtime experiment (separate design decision)

**Deliverables:** evaluate one small interpreter build targeting the guest ABI, measure module/runtime RAM footprint and VM compute cost; implement a trivial sensor/control example.

**Accept:** same VM engine runs interpreter-generated guest behavior; no special Python CPU accounting; incompatibilities with standard Python libraries explicitly documented. **Do not silently invent a Python subset without review.**

### Optional milestone 8 — JIT/backend optimization

After profiling, introduce a JIT/code cache only if performance tests justify it. Interpreter remains the reference **and the mandatory debugger backend**. Compare VM side effects, traps, logical CU billing and tick-boundary device events between backends on the same test programs. Source-level debugger hot-switching from an active JIT frame is explicitly not required.

**Implementation instruction:** deliver milestones incrementally. **Do not attempt every milestone in one large commit.** At each milestone provide changed files, build/run steps, tests, known limitations, and any ADR requiring human approval.

---

## 11. Mandatory acceptance scenarios

| ID | Test | Expected |
|---|---|---|
| A01 | RAM load/store of various widths | little-endian, correct values |
| A02 | Guest C volatile pointer -> controller MMIO | controller command/register updated, no accidental RAM write |
| A03 | controller transaction -> connected sensor | result appears after specified tick boundary, not direct sensor MMIO |
| A04 | high unmapped address | controlled fault, no host crash |
| A05 | RAM access crosses committed boundary | controlled fault, no partial store |
| A06 | MMIO unaligned/unsupported access | documented device fault |
| A07 | `memory.grow` under/over cap | correct page count/failure behavior |
| A08 | pure compute infinite loop | uses budget, then suspends; CLI remains responsive |
| A09 | resume after budget exhaustion | guest output equals uninterrupted reference semantics |
| A10 | 100 VMs with shared module | code shared safely, mutable state isolated |
| A11 | fixed scenario, worker counts 1/2/4 | equal guest-visible results and checksums |
| A12 | conflicting commands | deterministic stable resolution |
| A13 | unsupported WASM feature/module | validation error; no execution |
| A14 | CLI without Axmol | runs, tests, profiles and emits JSON |
| A15 | asynchronous host response | delivered at defined later tick, no worker blocking |
| A16 | guest trap during tick | other VMs continue; fault recorded |
| A17 | same trace replayed | identical guest-visible checksums |
| A18 | Active polling loop | consumes budget; stops at budget exhaustion without hanging host |
| A19 | LowPower polling loop | executes instructions with reduced deterministic budget, not zero instructions |
| A20 | Sleep via power MMIO | stops after current instruction; preserves PC, stack, RAM and registers |
| A21 | RTC alarm while sleeping | RTC advances; wake at documented boundary; guest resumes |
| A22 | device IRQ during Sleep | event wakes eligible VM; pending bit preserved until acknowledgement |
| A23 | interrupt already pending on Sleep entry | no lost wakeup or indefinite sleep |
| A24 | no wake source present | sleeping VM consumes no guest compute and is not executed by worker |
| A25 | power-mode rapid toggling | cannot mint free budget or accumulate unlimited unused credits |
| A26 | same mixed power/wake workload on 1/2/4 workers | same guest-visible events, RTC values, compute costs and checksum |
| A27 | wall/game time mode changes in host adapter | RTC follows completed simulation ticks, not wall clock or CPU rate |
| A28 | `sleep(0.001)` below tick duration | no sub-tick wake; earliest next eligible boundary |
| A29 | Guest switches CPU power mode repeatedly | operating points fixed by CPU model; CU change from next tick, no extra credits |
| A30 | controller correctly configured, pins connected | transaction completes at tick boundary with valid response |
| A31 | incompatible pins / wrong controller address | stable documented protocol error, NACK or timeout; no fake response |
| A32 | GPIO bit-banging demands sub-tick edges | unsupported behavior is explicitly documented; no timing guarantee |
| A33 | DMA active: Guest accesses valid buffer | no fault solely for Busy; intermediate values deliberately not asserted |
| A34 | DMA receive successfully completes | entire payload committed before completion status/IRQ is exposed |
| A35 | DMA error or conflicting write during transfer | status safe/defined; data not promised; host memory unaffected |
| A36 | protocol and DMA on workers 1/2/4 | identical completed payloads, statuses, IRQ ordering and checksums |
| A37 | enable JIT globally, start debug VM | debugger VM uses interpreter; optional optimization does not alter debug execution |
| A38 | CPU model power config | guest can select legal mode, cannot alter CU/power table, voltage or clock |

A15–A17 and A20–A38 can be marked deferred until their matching milestone; define the cases early as pending integration tests. A18/A19 belong with instruction accounting, and A20–A26 must pass before claiming power/wake scheduling is stable.

---

## 12. Performance benchmark plan

Benchmarks must report **host wall time separately from guest simulated compute time**.

Benchmark categories:

1. A: 100+ robots running tiny control loops dominated by sensor/MMIO operations.
2. B: 100+ robots doing substantial integer/path-planning-like computation without host calls.
3. C: 100+ robots with mixed device traffic, communication events, Active busy-waiting, LowPower busy-waiting, and Sleep/RTC/IRQ wake patterns.
4. D: repeated deployment of the same WASM module to test sharing of decoded/compiled module metadata.
5. E: worker scaling 1, 2, 4, 8 where available, with fixed simulated workloads.
6. F: protocol controller transactions with correct/incorrect pin mapping; IRQ wake and many sleeping peripherals.
7. G (deferred): DMA receive/transmit of 64B/4KiB/64KiB chunks, completion visibility and throughput.

Record: robots, guest instructions, simulated compute units, simulated ticks, host milliseconds, average/max tick processing time, peak host memory, guest RAM, worker scaling efficiency, sleeping-VM count, per-mode simulated durations, alarm/IRQ wake counts, and test-host hardware. Avoid setting arbitrary performance pass thresholds until representative workloads have been profiled.

---

## 13. Open questions requiring author decision (do not block early milestones)

1. Exact stable guest ABI / whether to use standalone freestanding imports, minimal WASI subset, or a custom libc profile.
2. Exact supported WASM extensions and what compiler flags the packaged build wrapper will set.
3. Whether device ranges can be remapped at runtime, and what happens if a device is hot-unplugged while a program runs.
4. Long-term save/load semantics for live call stacks, guest heap and pending asynchronous requests.
5. Python runtime selection and which pure-Python modules are supported in the game VM.
6. What host AI/network services are exposed to the VM, how request completions are serialized, and which permissions apply.
7. Whether the physical MMIO address layout is fixed for every computer type or programmable via a hardware description.
8. Whether the game world itself is simulated on a dedicated thread, or stepped synchronously by the Axmol adapter.
9. Final logging/debugging level and whether full source-level debugging is worth a later DAP adapter.
10. Difficulty-specific CPU/RAM budgets and template availability — **game design, intentionally excluded here**.
11. Final RTC unit/resolution and concrete register map, coherent 64-bit read/write protocol and alarm acknowledgement rules.
12. Full guest interrupt-handler ABI and priority/preemption behavior (v1 requires deterministic pending+wake; ISR delivery may be deferred).
13. Whether a world tick eventually supports sub-tick event execution; v1 deliberately uses boundary wake latency.
14. Exact developer-defined CU-throughput/power operating points, mode transition accounting and optional costs (no frequency/voltage emulation).
15. Concrete supported electrical/logical pin categories, controller protocol profiles and which protocol(s) ship in v1.
16. Whether game topology supports runtime re-wiring/hot-swap; changes must take effect at deterministic barriers.
17. DMA controller/descriptor register layout and data-transfer bandwidth; completion-only visibility contract is already fixed.
18. SDK ease-of-use layers (default drivers/templates, their unlock rules) — gameplay decision, not VM execution semantics.

---

## 14. Coding Agent task prompt

> Implement this repository according to `robot_simulator_spec.md`, beginning **only with Milestone 0 and Milestone 1**. Use C++20/CMake. The simulator core must be independent of Axmol and usable from a CLI. Keep each robot VM isolated; build the RAM/controller-MMIO bus and logical pin-topology contract with strict checked address semantics and tests. Do not add gameplay rules, JIT, DMA engine, full SPI/I²C/UART stacks, a Python interpreter, a full WASI runtime, guest virtual memory, sub-tick event processing or GUI until their designated milestones. Provide register/API placeholders for RTC, interrupt and power management, but defer full event-driven wake integration to Milestone 4. Document every provisional design assumption in an ADR. After completing the milestone, report changed files, commands to build/test, acceptance tests and unresolved design questions. Await further instruction before advancing to Milestone 2.

---

## Changelog

- **v0.3 (2026-11-17):** Locked abstract compute-throughput/power points (no frequency/voltage model); controller-MMIO plus logical pins, bus protocols and tick-boundary peripheral transactions; added DMA completion-buffer contract (safe but unspecified mid-transfer accesses) and deferred milestone; fixed debug VMs to interpreter; explicitly rejected sub-tick wakeups and specified short sleeps at next boundary; expanded acceptance tests and CLI schemas.

- **v0.2 (2026-11-17):** Added Active/LowPower/Sleep CPU semantics, RTC based on authoritative world time, interrupt pending/wake rules, budget policy, event-driven sleeper scheduling, CLI/profiling and acceptance tests. Replaced guest-visible `yield` design with optional hardware-oriented power/RTC wrappers; clarified tick-boundary wake granularity.

---

## Reference reading (normative background)

- WebAssembly Core Specification: https://www.w3.org/TR/wasm-core/ — standard instruction semantics, validation, trap conditions and linear memory. The proposed physical MMIO extension is *deliberately non-standard*.
- WASI SDK: https://github.com/WebAssembly/wasi-sdk — example of established C/C++ -> WASM toolchain; a custom freestanding guest ABI may be a better fit than full WASI for the initial robot program.

