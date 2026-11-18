# Robot Simulator — Peripheral Integration Design

**Version:** 0.4 draft · **Date:** 2026-11-18  
**Implementation scope:** M6B (Peripheral Bridge), M6C (DMA integration), M6D (CLI Mock Host and integration tests)  
**Baseline:** `robot_simulator_spec_v0.3.md` · Existing implementation reportedly through M6A  
**Status:** Design contract for Coding Agent; **not** a claim that these APIs already exist.

## 0. Executive decisions

1. The simulator is an independent C++20 library. Axmol and the CLI are hosts; they share **one** simulator and **one** communication/DMA semantics implementation.
2. The simulator owns WASM VM, Guest physical RAM and MMIO, pin topology validation, controllers, protocol transactions, timing, IRQ, RTC, CPU power, and DMA. The **host owns device-specific game behavior** (scanner samples, motor/arm/world physics, mining, etc.).
3. The host never receives a mutable Guest RAM pointer and never sets Guest IRQ/status registers. It supplies device capabilities, responses, and independent events. The simulator checks wiring, protocols, and time ordering.
4. **No sub-tick events.** Local CPU-visible controller register writes take effect as specified during VM execution; external device changes and transaction completions become Guest-visible only at prescribed tick input boundaries.
5. One VM is executed by at most one worker at a time, and on one worker per tick. Host-facing bridge calls occur **only on a coordinator thread at tick barriers**, never on VM workers.
6. M6A is already implemented according to the project owner; **audit and reuse it**, preserving the public DMA completion contract. Do not replace or duplicate DMA.
7. Debug VMs use the interpreter even if JIT is later enabled. JIT, Python Runtime, full GPIO waveform simulation, and real-world game physics are **out of scope**.
8. Real CPU models, capacities, protocol support and bandwidth values are injected via host-created configuration. The simulator contains no gameplay hardcoded tiers.

### Non-goals

- No world-space scanner implementation or robot locomotion in simulator core.
- No wiring voltage, individual SPI/I²C clock edges, signal integrity or reliable bit-banging with GPIO.
- No requirement for packet visibility during an incomplete DMA transfer.
- No new UI/editor or Axmol integration required to pass M6B–M6D; only a pure C++ host adapter example.
- No source-level debugging adapter, JIT, guest MMU, guest multithreading, or hot reload of hardware mid-tick.

## 1. Existing implementation audit — **mandatory first step**

Before editing any implementation, produce a short `docs/M6B_BASELINE_AUDIT.md` containing:

- Paths and public contracts for `Simulation::step`, VM ownership/scheduling, MMIO controller interface, RAM/bus, interrupt, power, and RTC devices.
- The real M6A DMA entry points, transfer direction support, completion and IRQ ordering, memory checks, serialization, and existing tests. Identify any unimplemented assumptions from v0.3.
- Existing CLI commands, JSON config schema and scenario fixtures.
- Existing per-tick phase order and whether multiple VMs can share an external device/controller.
- Required **adapter** methods and gaps; preserve previously passing tests. Do not assume names in this document exist in the repository.

If existing semantics conflict with a normative rule here, report it in an ADR and make the smallest compatibility-preserving change; do not silently alter guest ABI or save-file formats.

## 2. Responsibility and ownership map

| Resource | Owner | Access policy |
|---|---|---|
| VM execution state, RAM, CPU CU, local controller MMIO | Simulator / VM | VM worker during execute; coordinator at safe points |
| Logical pins, controller capability profile, protocol validation | Simulator | Host supplies immutable profile/topology at safe points |
| In-flight transactions, bandwidth allocation, completion ordering | Simulator | Coordinator owns shared mutable state |
| Guest DMA and interrupt state | Simulator | Coordinator commits writes at safe points; Guest MMIO access under VM ownership |
| External device identity and state (scanner, motor, mining logic) | Host | Host-only, updated at controlled tick boundary |
| Axmol nodes, game ECS, world physics | Game Host | Never accessed from simulator workers |
| CLI scripted mock device state | CLI Host | Same bridge contract; no duplicate protocol engine |

`DeviceId` and `VmId` are **stable opaque integer handles**, not host pointers. A device may be external to a specific robot; controller and connection handles must be namespaced appropriately to avoid collisions.

## 3. Interface boundaries

### 3.1 Separation of concepts

- **MMIO controller:** local CPU-visible register writes/reads, FIFO and status, which may be immediate **within the same VM** as defined by its ABI.
- **Protocol transaction:** accepted command originating from a controller; checked against supported controller capability, pin topology, device protocol and availability.
- **Host request:** normalized, immutable request emitted only after simulator-side validation; host determines device-specific response semantics.
- **Host readiness:** host has computed or produced response bytes. This does **not** mean communication/DMA is complete or Guest-visible.
- **Wire/completion:** simulator charges abstract bandwidth/latency and commits any DMA data, then publishes completion status and IRQ at a defined input boundary.
- **Unsolicited event:** host-originated input change or data-ready, routed to simulator controller/IRQ logic; never an unvalidated direct Guest interrupt.

### 3.2 Suggested C++20 DTOs (compile-oriented example; adapt existing names)

```cpp
using VmId = std::uint64_t;
using DeviceId = std::uint64_t;
using TxId = std::uint64_t;
using Tick = std::uint64_t;
using ControllerId = std::uint32_t;

struct Endpoint {
    VmId vm;
    ControllerId controller;
};

enum class Protocol : std::uint8_t { AbstractV1, Spi, I2c, Uart, Gpio };

enum class TransferKind : std::uint8_t { Command, Read, Write, Duplex };

// Protocol-specific parameters MUST use tagged variants, never opaque
// flags whose meaning changes across protocols.
struct AbstractParams { std::uint32_t address; };
struct SpiParams { std::uint32_t chip_select; std::uint8_t mode; };
struct I2cParams { std::uint16_t address; bool ten_bit; };
struct UartParams { std::uint32_t channel; };
using ProtocolParams = std::variant<AbstractParams, SpiParams, I2cParams, UartParams>;

struct PeripheralRequest {
    TxId id;
    Endpoint source;
    DeviceId target;
    Protocol protocol;
    TransferKind direction;
    ProtocolParams params;
    std::vector<std::uint8_t> tx;
    std::uint32_t expected_rx_bytes;
    Tick accepted_tick;
    std::uint64_t source_seq; // VM-local order, assigned by simulator
};

enum class HostResultCode : std::uint8_t {
    Success, DeviceBusy, DeviceError, NoResponse
};

struct PeripheralResponse {
    TxId id;
    HostResultCode result;
    std::vector<std::uint8_t> rx;
    Tick earliest_ready_tick;  // host result ready; NOT Guest completion
};

enum class EventKind : std::uint8_t {
    InputChanged, DataReady, DeviceFault, DeviceDisconnected
};
struct PeripheralEvent {
    DeviceId device;
    EventKind kind;
    std::vector<std::uint8_t> payload;
    Tick earliest_delivery_tick;
    std::uint64_t host_seq;
};
```

All buffers are owned values (or are copied before the call returns). Use bounded sizes from host config. Ensure no asynchronous use of a temporary `std::span`, VM pointer, raw device pointer, or Axmol pointer. `std::variant` requires `<variant>`; represent `Gpio` events separately if not a transaction in the actual codebase.

### 3.3 Host exchange contract

Prefer **pull/push batches at a barrier** over virtual callbacks from workers:

```cpp
struct HostTickInput {
    Tick tick;
    std::vector<PeripheralResponse> completed_work;
    std::vector<PeripheralEvent> events;
};
struct HostTickOutput {
    Tick tick;
    std::vector<PeripheralRequest> requests;
    std::vector<WorldCommand> world_commands; // if existing core emits them
};

// The game/CLI coordinates these calls; the simulator never calls Axmol.
class PeripheralBridge {
public:
    void queueHostInput(HostTickInput input); // only for future input tick
    HostTickOutput collectOutputAtBarrier();  // ordered, owned values
};
```

These are **conceptual APIs**: reuse the repository's current `Simulation::setInputs` / `step` / `StepResult` when possible. No need to add a second world-tick scheduler. The externally visible invariant is more important than exact C++ method spelling.

Recommended pipeline:

1. At tick-start coordinator, deliver only validated inputs whose `earliest_*_tick <= N`, subject to the deterministic visibility rule.
2. Parallel VM execution; controller-local MMIO state changes immediately as its register ABI specifies; requests are collected in per-VM ordered buffers.
3. At tick-end barrier, validate/resolve connection and protocol, assign `TxId`, and publish a **stable ordered** set of requests. Do not send invalid requests to host.
4. Host advances gameplay device state and returns ready responses/events using the input API at a **future** input boundary.
5. Coordinator advances protocol communication/bandwidth and DMA; **commit destination bytes before Completed/IRQ**; expose all external results no earlier than next defined input boundary.

**Minimum latency rule:** An external request accepted during Tick N must never yield Guest-visible response before Tick N+1, even if Host responds immediately in the same coordinator phase. A later response uses later tick. Transactions may require extra ticks due to bandwidth/configured delays. One-step phase conventions must be documented and tested; avoid off-by-one inconsistencies.

**Important:** The host may require time to calculate device-specific results. Host readiness and protocol transfer are **separate gates**; the simulator cannot mark a transaction successful before both have completed. If a Host returns error, follow the controller's specified error path.

### 3.4 Transaction lifecycle and ID policy

```text
Created (controller-local)
  -> Accepted -> AwaitingHost -> HostReady -> Transferring
  -> CommitPending -> Completed
              \-> Failed | TimedOut | Cancelled
```

Transitions do not have to consume separate ticks, but result visibility does. `TxId` is unique within the simulation session and never reused until a documented reset; duplicate or unknown response IDs are rejected and logged, not attached to a random transaction. Each accepted transaction has exactly one terminal outcome. Repeated final responses are rejected idempotently (no duplicate DMA or IRQ).

Cancellation cases: controller reset, VM removal, device removal, protocol disconnect, simulation teardown and explicit transfer cancel. Specify which controller status/IRQ is produced; in-flight DMA writes must not happen after cancellation. Avoid stale ID reuse by using monotonically assigned IDs / generation counters.

### 3.5 Device registration and wiring

- Host registers `DeviceId`, capability descriptor (supported protocol variants, protocol-specific addressing, pin roles, max payload, device bandwidth cap, powered/available state) and Host-owned opaque instance mapping.
- Host supplies `PinConnection` values for logical endpoints: `(VM, controller, pin)` to `(Device, pin)`; the simulator stores a graph and validates pin-mux, chip-select/address, direction, controller capability and power rules.
- **Hardware capability** comes from host configuration; **active protocol selection** is Guest MMIO programming; **device behavior** is provided by host responses/events.
- Registration, removal and rewiring occur only when quiescent at a tick barrier. No v1 hot-swapping in the middle of an active transaction.
- Existing transactions involving a removed device are deterministically cancelled or failed according to controller ABI; status/result is never silently attached to a new device reusing an ID.
- A generic abstract protocol controller is sufficient for M6B. Do not silently claim realistic SPI/I²C/UART implementations if current M4A only supports abstract transactions. Add a protocol only if its tested controller ABI is present.

### 3.6 Deterministic ordering

- Order accepted requests by `(accepted_tick, source.vm, source.controller, source_seq)`; stable unique tie-breaking and transaction IDs are assigned **after merge**, not by racing workers.
- Order host input events by `(earliest_delivery_tick, device, host_seq)`; validate host monotonic sequences or assign coordinator receive-order deterministically.
- Resource conflicts (shared controller, same addressed device, bus capacity) resolve in documented stable order; never depend on mutex acquisition or worker completion order.
- Worker counts 1, 2 and 4 with identical program/config/seed/host-script must produce identical Guest-visible state, transaction ordering, IRQ sequence and trace checksum.
- Invalid inputs, duplicate completions, oversized payloads, missing devices, wrong protocol versions: structured errors; no host crash or undefined behavior.

## 4. Timing model and flow control

- Tick resolution only; no sub-tick simulation, byte-level waveform, analog properties, or cycle-accurate arbitration.
- Fixed tick duration comes from simulation configuration. Controller/device bandwidth values are provided at VM/controller creation; in-game mode changes may affect **future** transactions only as defined by registers.
- For a transfer with `N` bytes and configured `B` bytes / **game second**, use fixed-point/integer accounting, e.g. accumulate `B * tick_us` and divide by 1,000,000, carrying the remainder. Define what counts as transmitted bytes (payload plus any protocol overhead). M6B may use abstract payload bandwidth; per-protocol overhead can be a versioned extension.
- Completion is at a tick boundary; even zero-length work cannot become visible inside the originating VM-execute phase.
- A controller is either single-inflight (recommended M6B) or has a specified queue depth. Busy/overflow/status must be defined; no implicit infinite queue.
- Sleep and RTC keep advancing by world simulated ticks, irrespective of Host wall time or Guest CPU compute mode.
- Peripheral responses can be delayed by host `earliest_ready_tick`; no VM worker can block on host/world computation.
- If a Host is temporarily absent, in-flight transactions remain pending until configured timeout/cancel; no implicit success.
- Configurable timeout rules must be deterministic and measured in simulation ticks or world time (not wall-clock seconds).

### 4.1 CPU-visible controller state

- A Guest write to a local controller config register updates local state immediately in program order (if the controller ABI says so).
- An accepted command should latch the relevant config/address/transmit payload at submission. Subsequent Guest writes do not mutate an accepted transaction unless explicitly defined by controller ABI.
- Busy/completion status is only published per ABI; external device result and IRQ not visible until next permitted input boundary.
- Faults from illegal MMIO access follow existing Guest fault rules; wiring/protocol errors set controller status/NACK/timeout, **not** an unrelated WASM trap.

## 5. DMA integration (M6C) — preserve M6A

M6A is reportedly implemented. M6C is an **adapter/integration** milestone; do not rewrite DMA to fit a hypothetical new engine.

### 5.1 Data ownership and copy points

1. `Host` produces **owned response bytes**. It never obtains or writes guest RAM pointers.
2. Simulator controller stores/queues the response and accounts for abstract transmission/bandwidth.
3. Controller forwards accepted data to M6A DMA service using a **validated Guest physical RAM address, length, and direction**.
4. At a coordinator-owned safe point, M6A commits data (single copy at completion is acceptable). No simultaneous VM worker RAM mutation.
5. Only after successful validation+commit does the simulator publish `Completed` status/IRQ at the defined input boundary.

The host device's "data ready" event and DMA "transfer complete" are distinct. Do not issue a DMA success interrupt merely because host computed the payload.

### 5.2 Explicit buffer contract (normative)

- While `Busy`, CPU reads and writes to a **valid** mapped RAM buffer never fault solely due to transfer state; content, partial progress, and conflicting-write effects are **unspecified**.
- Successful receive yields the full expected payload **provided there were no conflicting guest writes** during the transfer. Completion status / interrupt cannot precede the data commit.
- Host must not access Guest RAM directly, even for zero-copy performance optimization in v1.
- Transmit source should be prepared before launch and left untouched until terminal completion; mid-transfer mutations make output unspecified.
- Failure, timeout or cancel does not promise payload validity, but must report status safely; no stale post-cancel writes.
- Invalid physical address, length overflow or unmapped location follow M6A validation/fault/status rules. Busy is **not** permission to access unmapped memory.
- Repeatability within a fixed simulator/version/config must hold even where intermediate Guest buffer values are unspecified.

### 5.3 Integration boundary

Prefer a narrow adapter such as:

```cpp
class IDmaTransferSink {
public:
    virtual ~IDmaTransferSink() = default;
    virtual DmaSubmissionResult queueRx(VmId vm, std::uint32_t dst,
        std::span<const std::uint8_t> bytes, TxId tx) = 0;
    virtual DmaSubmissionResult queueTx(VmId vm, std::uint32_t src,
        std::uint32_t length, TxId tx) = 0;
};
```

Use `std::span` only for immediate call input; adapter must copy data or establish explicit immutable ownership before returning. Reuse existing DMA interfaces if they already express the required contracts.

### 5.4 Mandatory cases

- Transmit and receive paths; 0/1/256/4096-byte payloads and configured buffer limits.
- Data committed strictly before success status/IRQ; validate from Guest C program after wake.
- Guest access to active in-range buffer never faults **because** Busy (do not assert value).
- Error/timeout, device disconnect, controller reset and VM removal cancel safely.
- Repeated/stale/unknown Host completion does not duplicate DMA writes/IRQ.
- Active DMA on Sleeping CPU can complete and wake eligible VM.
- Identical outcomes and traces with worker counts 1/2/4.

## 6. CLI Mock Host (M6D)

The CLI should **test interface behavior**, not replicate game logic.

### 6.1 Mock capabilities

- Register fixture devices with capabilities/pin wiring (or intentionally bad wiring).
- Match transaction request/response using protocol-appropriate values and deterministic scripted sequences; avoid ambiguous global "next response" matching.
- Script deferred response, `Busy`, errors, NACK/no-response, timeout, disconnection and unsolicited events.
- Verify expected requests and result bytes, status flags, interrupts, DMA, Sleep/wake and exact tick visibility.
- Capture machine-readable trace with transaction ID, source VM/controller, target device, protocol, tick and final outcome.
- No game map, physics, scanning algorithm, items, Axmol dependency, or GPU requirement.

### 6.2 Suggested fixture schema (versioned)

```json
{
  "schema_version": 1,
  "tick_rate_hz": 30,
  "seed": 1234,
  "hardware": "fixtures/hardware/minimal_controller.json",
  "programs": [{"vm_id": 1, "path": "build/driver_fixture.wasm"}],
  "mock_devices": [{
    "device_id": 70,
    "protocol": "abstract-v1",
    "responses": [{
      "match_tx_hex": "0102",
      "rx_hex": "aabbccdd",
      "ready_after_ticks": 2,
      "result": "success"
    }]
  }],
  "assertions": [
    {"kind": "transaction_count", "value": 1},
    {"kind": "dma_rx_hex", "vm_id": 1, "address_hex": "00001000", "value": "aabbccdd"}
  ]
}
```

The fixture is **illustrative**. The agent should reuse/extend the existing CLI JSON parser/schema, not break it. `ready_after_ticks` is counted from a documented reference tick (the request's accepted tick is recommended); keep all time references consistent.

### 6.3 CLI commands

Preserve existing `validate`, `run`, `profile`, `test` syntax where implemented. Add optional scenario/fixture args, e.g.:

```sh
robot-sim run --program build/driver_fixture.wasm \
  --robot fixtures/hardware/minimal_controller.json \
  --scenario fixtures/scenarios/bridge_happy_path.json \
  --ticks 20 --workers 1 --trace out/bridge.trace.json
robot-sim test tests/integration/peripheral_bridge/
```

A nonzero exit code signals fixture assertion failure, malformed input or simulator fault. Include structured diagnostics with Tick, VM, Device, TxId and controller IDs. If the repository has different command names, preserve them and document equivalents.

## 7. Deliverables and staging

### M6B — Peripheral Bridge (implement first)

**Files/components (names flexible):** versioned DTOs and validation, host registration, topology, transaction pipeline, deterministic batch input/output, abstract controller integration, test fixture host; ADR for ordering/timing semantics.

**Acceptance:** A Guest C program writes controller MMIO -> valid connected mock request -> Host-owned response -> controller completion visible no earlier than next tick; wrong pins/config reject; 1/2/4-worker trace equivalent; no Axmol dependency; all old M0–M6A tests pass.

**Do NOT do yet:** new DMA engine, full SPI/I²C/UART waveforms, Python, JIT, game physics.

### M6C — Bridge-to-DMA (after M6B)

**Files/components:** adapter to existing M6A DMA, integration tests and phase ordering; no Host guest-memory pointer exposure.

**Acceptance:** 4 KiB Host response -> controller -> existing DMA -> RAM -> Completed/IRQ -> Guest reads exact data; Busy-buffer tests assert safety not contents; disconnect/cancel tests; 1/2/4 workers equal; no M6A regression.

### M6D — CLI Mock Test Host (after M6C)

**Files/components:** versioned fixtures, scripted error/delay/event inputs, trace/assertion format, CLI examples and developer docs.

**Acceptance:** A non-graphics CLI test suite covers valid/invalid pins, delayed readiness, error, IRQ, RTC-wake, DMA, fault injection, stale completion and multi-VM shared-device contention, with deterministic replay/trace. Existing CLI commands still work.

### Required end-of-milestone report

Provide: file changes, public API signature(s), new schema fields, command lines to build/run, test outputs, performance regressions (if measured), known limits, backward-compatibility notes and open ADRs. Do not start M6C/M6D until the preceding milestone passes.

## 8. Acceptance matrix

| ID | Milestone | Scenario | Expected |
|---|---|---|---|
| PB01 | M6B | valid wiring/controller configured | one stable accepted request, Host called only at barrier |
| PB02 | M6B | wrong/missing pins or incompatible protocol | deterministic controller failure, no Host request |
| PB03 | M6B | controller-local config write/read in same tick | local register behavior matches ABI |
| PB04 | M6B | Host reply ready immediately | not visible to Guest before Tick N+1 |
| PB05 | M6B | Host response delayed 3 ticks | exact documented tick visibility, no early IRQ |
| PB06 | M6B | duplicate/unknown response ID | rejected safely, no duplicated completion |
| PB07 | M6B | device unplug, reset or VM removed | in-flight operation cancelled without dangling refs |
| PB08 | M6B | workers 1/2/4; same inputs | same request order, Guest status and checksum |
| PB09 | M6B | shared external device contention | stable documented arbitration |
| PB10 | M6B | Host sends unsolicited data-ready | controller handles event, IRQ only if enabled |
| DI01 | M6C | received bytes through DMA | data committed before success status and IRQ |
| DI02 | M6C | Guest accesses active valid RX buffer | no Busy-only fault; intermediate values not asserted |
| DI03 | M6C | CPU sleeps while DMA in flight | device advances, IRQ wakes at next defined boundary |
| DI04 | M6C | failure, timeout, cancellation | no promised bytes or late DMA overwrite; safe status |
| DI05 | M6C | oversized/invalid DMA target | bounded failure, no Host/Guest RAM corruption |
| DI06 | M6C | 0/1/256/4096-byte transfers | successful lengths and data boundaries |
| DI07 | M6C | same scenario on 1/2/4 workers | same Guest-visible trace and checksum |
| CL01 | M6D | mock fixture valid driver | CLI passes without Axmol |
| CL02 | M6D | bad wiring/address/timeout fixture | CLI reports structured expected failure |
| CL03 | M6D | RTC and peripheral wake | deterministic wake tick and pending IRQ |
| CL04 | M6D | stale/duplicate Host message | rejected, trace records rejection |
| CL05 | M6D | repeated fixture run same seed | identical trace/checksum |
| CL06 | M6D | legacy CLI command and M6A tests | no regression |

## 9. Design constraints still open (do not silently decide)

1. Exact controller MMIO register layout and versioned ABI (`MMIO_ABI.md`).
2. Whether M6B implements only `AbstractV1` protocol or adapts already implemented SPI/I²C/UART; audit first.
3. Precise definition of protocol byte overhead, bit-to-byte throughput and nonintegral tick calculations.
4. Which errors set IRQ vs status-only, and cancellation behavior on unplug.
5. Whether input topology supports connecting a physical device shared by multiple robots.
6. How partial TX buffer writes are latched; favor snapshot at command acceptance and explicitly document.
7. Save-state serialization format and compatibility/versioning for in-flight host work.
8. Whether world command exchange is included in the same bridge batch (reuse existing world interface if present).

Coding Agent may propose defaults in ADRs and tests, but should **not** unilaterally alter fixed Guest-visible behavior or overwrite existing M6A contracts.

## 10. Prompt for Coding Agent

> You are implementing the next milestone of the standalone C++20 Robot Simulator. The repository reportedly passes M0–M6A. Read `robot_simulator_spec_v0.3.md` and `M6B_M6D_peripheral_integration_design.md`. **First audit** actual code and tests and write `M6B_BASELINE_AUDIT.md`; do not assume illustrative API names exist. Then implement **M6B only**: Host-defined peripheral behavior through a deterministic, tick-boundary, batch-oriented bridge; simulator-owned controller/connection validation; ownership-safe request/response/event DTOs; stable transaction ordering and lifecycle; one Mock abstract peripheral. Keep the VM engine, DMA implementation and existing CLI interface backward-compatible. Do not start M6C or M6D in this iteration. No Axmol dependency, no host Guest RAM pointers, no worker-to-Host callbacks, no sub-tick, JIT, Python or full gameplay device simulation. Add PB01–PB10 tests where applicable. Report build/test commands, test results, changed files, and every unresolved ABI choice before finishing.

---

**Relationship to baseline:** This document extends and refines v0.3 Sections 3.5–3.7, 5, 6–8, and milestones after M6A. It does not replace the validated existing WASM interpreter, power/RTC, DMA, or fixed-tick contracts.
