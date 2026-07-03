# Autonomous Tracking Drone — Zero to Hero

**STM32F4 Flight Controller + NVIDIA Jetson Orin Nano Companion Computer**

A complete bare-metal autopilot built from scratch across three lab experiments:
person detection, visual tracking, GPS waypoint navigation, RTH, geofencing, and automated simulation testing.

---

## Repository Structure

```
├── stm32/                  # STM32F4 bare-metal C firmware
│   ├── main.c              # Complete flight controller (Exp 1 + 2 + 3)
│   └── vision_additions.c  # Experiment 2 protocol additions reference
│
├── jetson/                 # NVIDIA Jetson Orin Nano Python code
│   ├── jetson_GPS_Simulation.py      # Dual-mode: upload mission / fly with vision
│   ├── vision_tracker.py   # Vision tracker with centroid tracker
│   └── ground_station.py   # Pre-flight ground station (telemetry display)
│
├── simulator/              # Desktop mission simulator (no hardware needed)
│   └── drone_simulator.py  # 2D map + telemetry panel + nav state machine
│
├── tests/                  # Automated CI test suite (Lecture 8)
│   ├── drone_sim_core.py   # Headless physics engine
│   └── test_missions.py    # 5 automated pass/fail tests
│
├── mission/                # Mission configuration
│   └── mission.json        # Waypoint mission template
│
└── docs/                   # Documentation
    └── WIRING.md           # Full pin connection diagram
```

---

## Experiments Covered

| Experiment | Description | Key Files |
|---|---|---|
| **Exp 1** | STM32 Autopilot from Scratch | `stm32/main.c` |
| **Exp 2** | Jetson Vision Integration | `jetson/vision_tracker.py` |
| **Exp 3** | Autonomous GPS Mission Planning | `jetson/jetson_exp3.py`, `mission/mission.json` |
| **Lecture 8** | Simulation & Automated Testing | `tests/`, `simulator/` |

---

## Hardware Required

| Component | Purpose |
|---|---|
| STM32F4 NUCLEO-F446RE | Main flight controller |
| NVIDIA Jetson Orin Nano | Companion computer (vision + mission) |
| IMX219 CSI Camera | Person detection |
| MPU6050 | IMU (gyro + accelerometer) |
| QMC5883L | Magnetometer |
| GPS module (NMEA) | Waypoint navigation |
| 4x ESC + Brushless motors | Propulsion |
| RC Transmitter/Receiver (iBUS) | Manual control |
| 3S LiPo battery | Power |

Full wiring details in [`docs/WIRING.md`](docs/WIRING.md)

---

## Quick Start

### 1. Flash STM32

Open `stm32/main.c` in STM32CubeIDE and flash to NUCLEO-F446RE.

### 2. Set Up Jetson

```bash
# Install dependencies
pip3 install ultralytics pyserial opencv-python

# Edit mission waypoints
nano mission/mission.json

# Upload mission to STM32 (on ground before flight)
python3 jetson/jetson_exp3.py upload

# Run vision tracker during flight
python3 jetson/jetson_exp3.py fly
```

### 3. Run the Simulator (no hardware needed)

```bash
pip3 install opencv-python numpy
python3 simulator/drone_simulator.py
```

Controls: `SPACE` = start mission, `R` = RTH, `B` = low battery, `G` = geofence, `Q` = quit

### 4. Run Automated Tests

```bash
python3 tests/test_missions.py
```

Expected output:
```
[PASS] Basic mission completion (3 waypoints)
[PASS] Low battery automatically triggers RTH
[PASS] Geofence breach automatically triggers RTH
[PASS] Hover action holds for correct duration
[PASS] Critical battery forces immediate LAND

5 / 5 PASSED
```

---

## System Architecture

```
┌─────────────────────┐         ┌─────────────────────────┐
│  Jetson Orin Nano   │         │   STM32F4 NUCLEO        │
│                     │         │                         │
│  IMX219 CSI Camera  │         │  MPU6050 + QMC5883L     │
│  YOLOv8 Detection   │         │  IMU Sensor Fusion      │
│  Centroid Tracker   │         │  PID Roll/Pitch/Yaw     │
│  Mission Upload     │◄─UART──►│  GPS Navigation         │
│  Telemetry Display  │         │  Geofencing + RTH       │
│                     │         │  Battery Monitor        │
└─────────────────────┘         └─────────────────────────┘
         │ Ethernet                        │ UART5
         │ RTSP Video                      ▼
         │                        ┌─────────────────┐
         └───────────────────────►│  SIYI A8 mini   │
                                  │  3-axis Gimbal  │
                                  └─────────────────┘
```

---

## UART Protocol (Experiment 2)

Custom framed binary protocol between Jetson and STM32:

```
[0xAA][0x55][msg_id][length][payload...][XOR checksum]
```

| MSG ID | Name | Direction | Payload |
|---|---|---|---|
| 0x01 | HEARTBEAT | Jetson → STM32 | 1 byte status |
| 0x02 | TRACK_CMD | Jetson → STM32 | offset_x, offset_y, confidence |
| 0x10 | ATTITUDE | STM32 → Jetson | roll, pitch, yaw, alt |
| 0x11 | GPS_DATA | STM32 → Jetson | lat, lon, speed |
| 0x12 | BATTERY | STM32 → Jetson | voltage, current |
| 0x20 | WP_UPLOAD | Jetson → STM32 | waypoint data |
| 0x21 | MISSION_CMD | Jetson → STM32 | start/stop |

---

## Navigation State Machine (Experiment 3)

```
IDLE → TAKEOFF → NAVIGATE → HOVER → ADVANCE → RTH → LAND
```

### Failsafe Triggers

| Trigger | Condition | Response |
|---|---|---|
| RC Signal Lost | > 2 seconds | RTH |
| GPS Fix Lost | > 5 seconds | Land |
| Battery Warning | < 10.8V | RTH |
| Battery Critical | < 10.2V | Land immediately |
| Geofence Breach | Outside 50m radius | RTH |
| Kill Switch | AUX2 < 1200 | Motors OFF |

---

## STM32 Pin Map

| Peripheral | Pins | UART/Timer |
|---|---|---|
| IMU (MPU6050 + QMC5883L) | PB6 SCL / PB7 SDA | I2C1 |
| GPS | PA9 TX / PA10 RX | USART1 @ 9600 |
| Debug printf | PA2 TX | USART2 @ 115200 |
| Jetson Link | PB10 TX / PB11 RX | USART3 @ 115200 |
| iBUS RC | PC11 RX | UART4 DMA |
| SIYI Gimbal | PC12 TX / PD2 RX | UART5 @ 115200 |
| ESC PWM | PA0 PA1 PA2 PA3 | TIM2 CH1-4 @ 50Hz |
| Battery ADC | PA0 | ADC1 CH0 |

---

## License

MIT License — free to use, modify, and distribute.

---

## Authors

Autopilot Lab Series — Experiments 1, 2, 3 and Lecture 8
Built with STM32CubeIDE (bare-metal C) and Python 3 on NVIDIA Jetson Orin Nano.
