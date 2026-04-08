# RID_Scanner

ESP32-S3 firmware that detects drone Remote ID (RID) broadcasts over WiFi and Bluetooth, following the ASTM F3411-22 standard. Outputs real-time JSON telemetry over UART to a downstream aggregator.

## Features

- WiFi promiscuous mode scanning (NAN Action Frames + Beacons)
- BLE 5.0 extended advertising and BLE 4.x legacy scanning
- ASTM F3411-22 protocol parsing (Remote ID v2, backward compatible v0-v1)
- Tracks up to 64 simultaneous UAVs
- Channel hopping across WiFi frequencies (500ms interval)
- Dual UART output — USB console + GPIO17 for aggregator
- Rate-limited JSON output (~1Hz per drone)

## Hardware

| Component | Details |
|-----------|---------|
| Board | ESP32-S3-DevKitC-1-N16R8 |
| Flash | 16MB |
| PSRAM | 8MB |
| WiFi | 802.11 b/g/n (promiscuous mode) |
| Bluetooth | 5.0 LE (legacy + extended advertising) |

### Wiring

| Pin | Function | Notes |
|-----|----------|-------|
| GPIO 17 | UART1 TX | Connect to DroneMonitor ESP32-P4 RX (GPIO3) |
| USB | UART0 | Console debug output |

## Project Structure

```
├── src/
│   ├── main.c              Core app — scanning coordination, JSON output
│   ├── opendroneid.c/h     ASTM F3411 RID protocol library
│   ├── ble_scanner.c/h     BLE scanning (Bluedroid, legacy + extended)
│   ├── wifi.c              WiFi promiscuous mode, NAN/Beacon parsing
│   ├── odid_wifi.h         IEEE 802.11 and NAN frame structures
│   └── CMakeLists.txt      Build dependencies
├── platformio.ini          PlatformIO build config
├── sdkconfig.defaults      ESP-IDF configuration (source of truth)
└── partitions.csv          Flash partition table
```

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/)
- ESP32-S3-DevKitC-1-N16R8 board
- USB cable for flashing and console output

### Build and Flash

```bash
pio run --target upload    # compile and flash
pio device monitor         # view serial output (115200 baud)
```

### Clean Build

```bash
pio run --target clean
pio run --target upload
```

## JSON Output Format

Each detected drone produces a JSON message on both UART0 and UART1:

```json
{
  "mac": "AA:BB:CC:DD:EE:FF",
  "rssi": -65,
  "iface": 4,
  "id": "drone-serial-id",
  "id_type": 1,
  "ua_type": 1,
  "lat": 30.049883,
  "lon": -81.826582,
  "alt": 100.5,
  "height": 50.2,
  "hspd": 5.3,
  "vspd": 0.0,
  "hdg": 180,
  "sts": 0,
  "op_id": "operator-id"
}
```

### Interface Types

| `iface` | Source |
|---------|--------|
| 1 | WiFi Beacon |
| 2 | WiFi NAN |
| 3 | BLE 4.x Legacy |
| 4 | BLE 5.0 Extended |

## Scanning Parameters

| Parameter | Value |
|-----------|-------|
| Max tracked UAVs | 64 |
| WiFi channel hop | 500ms |
| BLE scan (idle) | 500ms |
| BLE scan (active) | 1000ms |
| BLE output rate limit | 1Hz per drone |
| Status report interval | 60s |

## Related Projects

- [DroneMonitor](../DroneMonitor/) — ESP32-P4 aggregator, web dashboard, and backend API (receives this scanner's output)
