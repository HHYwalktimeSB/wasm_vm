# Headless trace format v1

`robot_sim_cli run PROGRAM --trace PATH --ticks N` records a single-robot session. `robot_sim_cli run PROGRAM --replay PATH` consumes recorded replies and IRQ inputs and checks the state checksum after each tick. Playback uses the supplied WASM file and the controller and CPU configuration captured in the trace. `--workers` can be changed on playback.

The line-oriented v1 header starts with `robot-sim-trace-v1 cu-v1`. Remaining fields record the WASM byte checksum, seed, Active and LowPower CU/s, tick rate, duration in ticks, RAM cap in pages, DMA byte cap, protocol ID, target, timeout and the four controller and peripheral pin assignments.

Each tick stores the tick number, simulator state checksum and reply count. Reply fields are request ID, status, response word, payload length and byte values. A final IRQ mask records the scripted external IRQs for the tick. The checksum covers RAM, MMIO state, interpreter continuation and scheduler state in stable robot order. The trace uses FNV-1a for reproducibility checks. Recording and playback are capped at 64 MiB.

This CLI trace captures the test peripheral session. The game can use the same deterministic Simulation API by persisting the creation configuration, initial modules and the WorldInput provided for each tick.

