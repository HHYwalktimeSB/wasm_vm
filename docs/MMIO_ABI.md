# Robot MMIO ABI v1

The game-facing Simulation uses the default controller window at `0x40000000`. The lower-level Bus constructor can relocate the window, and the SDK targets the default layout. Listed registers require aligned 32-bit access. RAM allows unaligned little-endian access within committed pages. Effective addresses are checked as 64-bit sums before classification.

| Offset | Access | Behavior |
|---|---|---|
| 0x00 | R/W | Power mode. 0 Active, 1 LowPower, 2 Sleep |
| 0x10, 0x14 | R | Low/high halves of the authoritative simulation timestamp in nanoseconds |
| 0x18, 0x1c | R/W | Stage the next alarm deadline low/high halves |
| 0x20 | R/W | Latched IRQ bits. Write ones to acknowledge each selected bit |
| 0x24 | R/W | IRQ wake mask |
| 0x28 | R/W | Read whether an alarm is armed. Write 1 to latch and arm the staged deadline, or 0 to disarm |
| 0x30 | R/W | GPIO placeholder |
| 0x100 | R/W | Target address |
| 0x104 | W | Queue a command and capture the target |
| 0x108 | R | 0 idle, 1 busy, 2 complete, 3 connection or peripheral failure, 4 timeout |
| 0x10c | R | Last successful response. It belongs to the current command when status is 2 |
| 0x120 | R/W | DMA guest RAM address |
| 0x124 | R/W | DMA size in bytes |
| 0x128 | W | Start DMA. 1 receives into RAM, 2 transmits from RAM |

The RTC timestamp stays constant throughout a guest execution slice. Both halves read during the same slice are coherent. At the first completed tick boundary at or after its deadline, a one-shot alarm disarms and latches IRQ bit 1. Reading either RTC register leaves pending IRQs latched.

IRQ bit 0 indicates ordinary protocol completion or failure, bit 1 indicates an RTC alarm, and bit 2 indicates DMA completion or failure. External IRQ masks, device results and alarms are merged at boundaries. Wake returns the CPU to Active and preserves pending bits. Sleep stops the current execution slice even when an eligible IRQ is already pending. The VM can run again on the next tick.

## Controller and DMA

The game opts into the abstract protocol when creating a VM. A connection assigns CLK and DATA to two distinct logical nets, each numbered 0 through 31. Requests require matching CLK/DATA nets at both ends and a matching target. A busy controller accepts one transfer at a time. A second start write faults. An unconnected or unconfigured route reports status 3 at a boundary. A valid route emits a request at the end of the producing tick. The game can return a completion at the start of a subsequent tick. A completion arriving at the configured timeout boundary takes priority over timeout.

The game enables DMA by configuring a positive maximum byte count up to 65,536. The start register validates the entire nonempty buffer against committed RAM and captures its address, size and direction. An invalid range faults before issuing a request.

A transmit request contains the complete RAM snapshot captured at the boundary where the game receives the request. The guest observes completion when the game subsequently acknowledges it. A successful receive writes the complete host payload to RAM before publishing completion and IRQ. A payload of the wrong length fails without committing bytes. The guest may read and write valid RAM while DMA is busy. Intermediate contents and the effect of conflicting writes are unspecified.

