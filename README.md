# Preemptive RTOS and Sensor Pipeline on ARM Cortex-M3

Bare-metal C++17 firmware for an ARM Cortex-M3 with a custom preemptive RTOS kernel, a
register-level UART driver, a HardFault handler, and a fixed-point Kalman filter.
It runs in the QEMU `mps2-an385` emulator, so no hardware is needed.

```
sensor_task (100 Hz, prio 3) --raw_q--> filter_task (prio 2) --log_q--> logger_task (prio 1)
   simulated IMU signal            Q16.16 Kalman filter             UART output + self-check
```

## What is implemented
- **Startup (bare metal):** hand-written vector table, linker script, `.data` copy, `.bss` zeroing,
  static constructors, no vendor HAL or CMSIS.
- **RTOS kernel:** fixed-priority preemptive scheduler with round-robin among equal priorities.
  Context switch in `PendSV` (assembly, saves r4-r11 to the task stack), 1 kHz `SysTick` tick,
  first task launched via `SVC`, drift-free `delay_until`.
- **Synchronization:** counting semaphores with real blocking (tasks leave the ready set) and a
  blocking bounded queue built on them. Critical sections save/restore `PRIMASK`.
- **Timeouts:** `Semaphore::take(ms)`, `Queue::push/pop(..., ms)` return `false` on expiry
  (`0` = non-blocking try). SysTick wakes timed-out waiters, so a stalled producer can never
  hang a consumer.
- **Watchdog and fault injection:** the filter task uses a 50 ms receive timeout. The sensor
  deliberately stalls for 300 ms mid-run, and the self-test requires the watchdog to detect it and
  the pipeline to recover with no lost samples.
- **Driver:** CMSDK UART driven directly through memory-mapped registers.
- **Fault handling:** `HardFault` handler selects MSP/PSP, prints stacked PC/LR, CFSR, HFSR, BFAR.
- **Fixed-point DSP:** scalar Kalman filter in Q16.16 (Cortex-M3 has no FPU).
- **Diagnostics:** stack-canary high-water marks per task, context-switch count, queue depth,
  end-to-end pipeline latency.

## Build and run
```
sudo apt install gcc-arm-none-eabi libstdc++-arm-none-eabi-newlib qemu-system-arm make
make run          # build and run in QEMU (exits automatically through semihosting)
make test         # run and assert RESULT: PASS
make fault-test   # trigger a deliberate bus fault to show the HardFault report
```

## Sample result (`make test`)
```
samples processed   : 500
RMS error raw       : 174
RMS error filtered  : 66
noise reduction     : 63%
watchdog timeouts   : 5           (injected 300 ms sensor stall detected)
max pipeline latency: 1 ms        (1 ms tick resolution)
context switches    : 1984
```
Firmware size: about 4.1 KB text, 0.4 KB data.

## Limitations
- The IMU is simulated (triangle wave plus uniform noise), not a real sensor.
- Semaphores have no priority inheritance, so priority inversion is possible in principle.
- Single core, no dynamic task creation after `start()`.
- `make test` runs in QEMU, so timing figures are emulated, not measured on silicon.

## Files
| File | Purpose |
|---|---|
| `src/startup.cpp` | Vector table, `Reset_Handler`, runtime init |
| `src/rtos.hpp/.cpp` | Kernel: scheduler, PendSV/SVC/SysTick, semaphores, queue |
| `src/uart.hpp/.cpp` | Register-level UART driver |
| `src/fault.cpp` | HardFault handler |
| `src/kalman.hpp` | Q16.16 Kalman filter |
| `src/main.cpp` | Application tasks and self-test |
| `linker.ld` | Memory map and sections |
