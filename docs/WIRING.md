# Wiring Guide — Autonomous Tracking Drone

## STM32 NUCLEO-F446RE Connections

### I2C1 — Sensor Bus (shared)
| STM32 Pin | Connects To | Purpose |
|---|---|---|
| PB6 (SCL) | MPU6050 SCL + QMC5883L SCL | Clock |
| PB7 (SDA) | MPU6050 SDA + QMC5883L SDA | Data |
| 3.3V | MPU6050 VCC + QMC5883L VCC | Power |
| GND | MPU6050 GND + QMC5883L GND | Ground |

Note: MPU6050 AD0 pin → GND (sets I2C address to 0x68)

### USART1 — GPS (9600 baud, interrupt-driven)
| STM32 Pin | Connects To |
|---|---|
| PA10 (RX) | GPS module TX |
| 3.3V or 5V | GPS VCC |
| GND | GPS GND |

### USART2 — Debug printf (115200 baud)
| STM32 Pin | Connects To |
|---|---|
| PA2 (TX) | USB-TTL adapter RX |

### USART3 — Jetson Link (115200 baud)
| STM32 Pin | Connects To | Note |
|---|---|---|
| PB10 (TX) | Jetson Pin 10 (RX) | TX crosses to RX |
| PB11 (RX) | Jetson Pin 8 (TX) | RX crosses to TX |
| GND | Jetson Pin 6 (GND) | Common ground — mandatory |

### UART4 — iBUS RC Receiver (115200 baud, DMA)
| STM32 Pin | Connects To |
|---|---|
| PC11 (RX) | RC receiver iBUS signal |
| 5V | RC receiver VCC |
| GND | RC receiver GND |

### UART5 — SIYI Gimbal (115200 baud)
| STM32 Pin | Connects To |
|---|---|
| PC12 (TX) | SIYI control cable RX |
| PD2 (RX) | SIYI control cable TX |
| GND | SIYI control cable GND |

### TIM2 — ESC PWM (50Hz, 1000-2000us)
| STM32 Pin | Connects To |
|---|---|
| PA0 (CH1) | ESC 1 signal |
| PA1 (CH2) | ESC 2 signal |
| PA2 (CH3) | ESC 3 signal |
| PA3 (CH4) | ESC 4 signal |

### ADC1 — Battery Voltage Monitor
| STM32 Pin | Connects To |
|---|---|
| PA0 | Voltage divider output |

Voltage divider: 10kΩ from LiPo+ to PA0, 3.3kΩ from PA0 to GND

---

## Jetson Orin Nano Connections

| Jetson Pin | Connects To | Purpose |
|---|---|---|
| Pin 8 (TX) | STM32 PB11 (RX) | Send tracking commands |
| Pin 10 (RX) | STM32 PB10 (TX) | Receive telemetry |
| Pin 6 (GND) | STM32 GND | Common ground |
| CSI port | IMX219 ribbon cable | Camera (15-pin flat) |
| USB-A port | SIYI A8 USB-C (via adapter) | Optional video/config |
| Ethernet | SIYI A8 Ethernet | RTSP video + UDP SDK |

---

## Power Connections

```
3S/4S LiPo (11.1V - 14.8V)
        │
        ├── Red (+)  ──→ SIYI A8 mini power connector (red)
        ├── Red (+)  ──→ 5V BEC input → Jetson 5V input
        └── Black(−) ──→ SIYI A8 mini power connector (black)
                                    │
                                    └── Common GND bus
                                        ├── STM32 GND
                                        ├── Jetson Pin 6
                                        └── All ESC GND
```

---

## RC Switch Mapping

| Channel | Switch | Position | Action |
|---|---|---|---|
| CH5 (AUX1) | 3-pos switch | > 1700 | Start autonomous mission / RTH |
| CH6 (AUX2) | 2-pos switch | < 1200 | Kill switch — motors OFF immediately |

---

## Connection Order (Safety — Every Time)

```
1. Connect all GND wires first
2. Connect signal wires (UART TX/RX, I2C)
3. Connect camera ribbon cable
4. Connect Ethernet between Jetson and SIYI
5. Apply LiPo power to SIYI gimbal → wait 30 seconds for boot
6. Power STM32 via USB
7. Power Jetson via BEC
```

**Never connect or disconnect signal wires while devices are powered.**

---

## CSI Camera Notes

- Blue/silver contact side of ribbon faces the board
- Only valid IMX219 sensor modes:
  - 3280x2464 @ 21fps
  - 3280x1848 @ 28fps
  - 1920x1080 @ 30fps
  - 1640x1232 @ 30fps
  - **1280x720 @ 60fps** ← used in code (fastest)
- 640x480 is NOT a valid mode — will always fail

---

## SIYI A8 mini Notes

- Power: 11-16.8V (3S or 4S LiPo), up to 12W / ~1A
- **Never power from Jetson or STM32 GPIO pins**
- Serial: 3.3V TTL — compatible with STM32 directly (no level shifter)
- Boot time: ~30 seconds after power on
- Default IP: 192.168.144.25
- RTSP: rtsp://192.168.144.25:8554/main.264
- UDP SDK port: 37260
