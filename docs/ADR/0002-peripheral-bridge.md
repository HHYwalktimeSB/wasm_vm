# ADR 0002 — Peripheral bridge ordering and timing

Status: provisional for M6B.

The game calls PeripheralBridge from its coordinator and owns the external device behavior. The bridge owns the Simulation and exposes snapshots and owned messages. The existing raw Simulation API and CLI trace v1 retain their contracts. Bridge calls require a quiescent coordinator boundary.

## Configuration and protocol

Register a device with PeripheralDevice, create a VM with RobotConfig and call connect with both IDs. The existing ControllerConfig defines the controller pin pair and its endpoint pin pair. The bridge verifies the registered endpoint's pins, capability and target before emitting a request. Controller ID 0 identifies the VM's existing abstract controller. The current MMIO ABI fixes the protocol to AbstractV1 and lets the guest program the target and submit commands.

Device IDs and accepted transaction IDs are monotonic within a bridge instance. Multiple robots may connect to a device. The first valid request in ascending Robot ID order reserves that device until a terminal outcome. Other contenders receive the existing controller failure status at the next boundary. Requests rejected during routing receive no transaction ID.

## Readiness, transport and cancellation

A request accepted in tick N becomes externally visible after that tick. The host may report readiness at N or later in the next step input. The effective ready tick is the later of reported readiness and the input boundary. Each eligible boundary adds the lesser of the controller and device rates in bytes per second to a credit counter. Completion requires accumulated credits of payload bytes times ticks per second. An ordinary command charges four bytes each for the outbound command and returned response. DMA charges its configured payload length. Credits are per transaction.

At each start boundary, dated events are delivered in (tick, device, host sequence) order. A DataReady event on an enabled, correctly wired device latches IRQ bit 3 on the connected controllers. The existing guest wake mask controls whether the IRQ wakes a sleeping VM. A disconnect cancels in-flight work before evaluating replies. A success reaching its transport target on the timeout boundary wins. Busy and device error use status 3, and NoResponse reaches the existing status 4 timeout. Stale, duplicate and malformed host inputs are returned in the rejected batch.

Explicit cancel and unplug deliver status 3 and the corresponding controller IRQ at the next boundary. VM removal drops its pending work, and the next output batch reports cancellation. A removed device ID is never assigned to a replacement. Guest reset and cancel registers remain an open ABI choice.

The bridge forwards terminal success or failure to the existing M6A completion mechanism. It preserves DMA's TX snapshot point, committed-range validation, and RX-copy-before-IRQ ordering. Zero-length DMA retains its existing bounds fault. Full DMA bridge acceptance scenarios belong to M6C. CLI scenario and trace integration belong to M6D.

## Remaining choices

Future protocol selection and addresses, richer pin topology, reset semantics, in-flight save-state format and world-command coalescing require separate ABI decisions.

