# Real-Time Data Visualization

**STM32 + FreeRTOS sensor acquisition with soft real-time visualization on Linux.**

## Objective

Design a lightweight embedded system that collects real-time sensor data on an RTOS for deterministic response, and visualizes it on Ubuntu (PREEMPT_RT kernel) with minimal latency.

## Architecture

```
[HC-SR04] → [STM32 Blue Pill + FreeRTOS] → [UART] → [Ubuntu PREEMPT_RT + SDL]
                    │
                    ├─ Producer Thread (High Priority): Sensor read + validation
                    └─ Consumer Thread (Low Priority): UART transmission
```

## Core Mechanism: Double Buffer Latest-Wins Mailbox

| Buffer | Role |
|--------|------|
| **Back Buffer** | Producer writes here (never blocks) |
| **Front Buffer** | Consumer reads here (always latest data) |

Atomic swap ensures producer never waits. Consumer synchronizes on swap, tolerating minor data drops to eliminate accumulated latency.

## Key Properties

- **Non-blocking Producer**: Critical sensor thread never waits
- **Latest-Wins Logic**: Consumer always displays most recent data
- **Deterministic Timing**: FreeRTOS priority-based scheduling
- **Soft RT Visualization**: PREEMPT_RT kernel + SDL rendering

## Tech Stack

| Layer | Technology |
|-------|------------|
| MCU | STM32F103C8T6 |
| RTOS | FreeRTOS |
| Sensor | HC-SR04 Ultrasonic |
| Comm | UART 115200 |
| Host | Ubuntu + PREEMPT_RT |
| Render | SDL2 |

## Testing

Compare RTOS vs Bare Metal latency. Target: `RTOS_latency / BareMetal_latency ≤ 1.8`

## Quick Start

```bash
# STM32
flash graduation_project via ST-Link

# Ubuntu
gcc -o sensor_monitor sensor_monitor_linux.c -lSDL2
./sensor_monitor /dev/ttyUSB0
```

## Structure

```
├── Core/           # STM32 firmware
├── Middlewares/    # FreeRTOS
├── PC_App/         # Linux SDL visualization
└── Drivers/        # HAL
```

---

*Undergraduate graduation project — Real-time embedded systems.*
