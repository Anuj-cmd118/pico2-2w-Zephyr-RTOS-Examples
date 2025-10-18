#  Zephyr RTOS Examples for Raspberry Pi Pico 2 / Pico 2W

This repository is a growing collection of **Zephyr RTOS projects and experiments** for the **Raspberry Pi Pico 2** and **Pico 2W**.  
It’s designed to help **students, hobbyists, and embedded developers** learn Zephyr RTOS concepts through **practical, ready-to-run examples**.

Each folder contains both the **source code** and **prebuilt firmware (`.uf2`)** files, so you can start experimenting instantly — even without setting up the full build environment.

---

## Contents

| Example | Description | Prebuilt Firmware |
|----------|--------------|-------------------|
| `blinky/` | Basic Zephyr app demonstrating GPIO toggling | ✅ `blinky/build/zephyr/zephyr.uf2` |
| `PID control/` | Closed-loop control using Zephyr threads and timing APIs | ✅ `PID control/build/primary/zephyr/zephyr.uf2` |
| *(more examples Coming soon)* | Networking, sensors, and advanced RTOS concepts | 🚧 Coming updates |

---

## 🚀 Getting Started

### Option 1: Flash Prebuilt Firmware
1. Hold the **BOOTSEL** button on your Pico 2 / Pico W and connect it via USB.  
2. Your board will appear as a USB drive.  
3. Copy any `.uf2` file (found inside the example’s `/build/.../zephyr/` folder) onto the drive.  
4. The device will reboot and start running the example.

### Option 2: Build from Source
If you have the Zephyr SDK and `west` installed:
```bash
west build -b rpi_pico2 path/to/example
west flash
