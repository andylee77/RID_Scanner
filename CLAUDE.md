# RID_Scanner — Project Rules

## Project Overview

This is **RID_Scanner** — an ESP32-S3 firmware that scans for ASTM F3411 Remote ID
drone broadcasts via WiFi (NAN + Beacon) and BLE (legacy + extended advertising).

- **Language:** C (ESP-IDF framework)
- **Build system:** PlatformIO + CMake
- **Hardware:** ESP32-S3-DevKitC-1-N16R8 (16MB flash, 8MB RAM)
- **Protocol:** ASTM F3411-22 (Remote ID v2)
- **Output:** JSON over dual UART (USB console + GPIO17 to aggregator)

## Key Files

| File/Dir | Purpose |
|----------|---------|
| `DEVLOG.md` | Developer reference — hardware, build commands, pin config |
| `src/main.c` | Core app — scanning coordination, JSON output, UART |
| `src/opendroneid.c/h` | ASTM F3411 protocol parsing (Intel ODID library) |
| `src/ble_scanner.c/h` | BLE 5.0 scanning implementation (Bluedroid) |
| `src/wifi.c` | WiFi promiscuous mode scanning, NAN/Beacon parsing |
| `src/odid_wifi.h` | IEEE 802.11 and NAN frame structure definitions |

## Project Structure

```text
src/
├── main.c              ← Core app (scanning, JSON output, UART)
├── opendroneid.c/h     ← ASTM F3411 RID protocol library
├── ble_scanner.c/h     ← BLE scanning (legacy + extended advertising)
├── wifi.c              ← WiFi promiscuous mode scanning
├── odid_wifi.h         ← 802.11/NAN frame structures
└── CMakeLists.txt      ← Build dependencies
```

## Build Environment

- **Framework:** ESP-IDF 5.5.2 via PlatformIO
- **Board:** esp32-s3-devkitc1-n16r8
- **Build:** `pio run` (compile) or `pio run --target upload` (flash)
- **Monitor:** `pio device monitor`
- **Clean:** `pio run --target clean`
- **Output:** `.pio/` directory (gitignored)

## Hardware Configuration

- **UART1 TX:** GPIO 17 (to DroneMonitor ESP32-P4 aggregator, 115200 baud)
- **WiFi:** Promiscuous mode, channel hopping every 500ms
- **BLE:** Bluedroid, BLE 5.0 extended advertising, scan interval 500-1000ms
- **Max tracked UAVs:** 64 simultaneous

## Conventions

- Source code lives in `src/` only
- Build outputs go to `.pio/` and `build/` (both gitignored)
- Generated `sdkconfig*` files (except `sdkconfig.defaults`) are build artifacts
- `sdkconfig.defaults` is the source of truth for ESP-IDF configuration

## Related Projects

| Repo | Location | Purpose |
|------|----------|---------|
| DroneMonitor | `C:\Users\Andy\Projects\DroneTracking\DroneMonitor` | ESP32-P4 aggregator — receives this scanner's UART output |
| Shared docs | `C:\Users\Andy\Projects\_shared\` | Cross-project documentation |
