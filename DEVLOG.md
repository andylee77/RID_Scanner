# RID_Scanner — Developer Reference

## Repo Layout

```text
RID_Scanner/
├── CLAUDE.md              ← Project rules (AI assistant guide)
├── DEVLOG.md              ← This file — developer reference
├── platformio.ini         ← PlatformIO build config
├── CMakeLists.txt         ← Root CMake config
├── partitions.csv         ← Flash partition table
├── sdkconfig.defaults     ← ESP-IDF default config (source of truth)
│
├── src/                   ← All source code (C, ESP-IDF)
├── include/               ← PlatformIO includes (empty)
├── lib/                   ← PlatformIO libraries (empty)
└── test/                  ← PlatformIO test directory
```

## Build Commands

```bash
pio run                     # Compile
pio run --target upload     # Compile and flash
pio device monitor          # Serial monitor (115200 baud)
pio run --target clean      # Clean build
```

## Hardware Pinout

| Pin | Function | Notes |
|-----|----------|-------|
| GPIO 17 | UART1 TX | To DroneMonitor aggregator (115200 baud) |
| GPIO 16 | UART1 RX | Configured but unused |
| USB | UART0 | Console debug output |

## Scanning Configuration

| Parameter | Value |
|-----------|-------|
| Max tracked UAVs | 64 |
| WiFi channel hop interval | 500ms |
| BLE short scan (idle) | 500ms |
| BLE long scan (active) | 1000ms |
| BLE pause between scans | 100ms |
| BLE output rate limit | 1Hz per drone |
| Status report interval | 60s |

## JSON Output Format

Each detected drone produces a JSON object on both UART0 (console) and UART1 (aggregator):

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

**Interface types (`iface`):**

| Value | Type |
|-------|------|
| 1 | WiFi Beacon |
| 2 | WiFi NAN |
| 3 | BLE 4.x Legacy |
| 4 | BLE 5.0 Extended |

## Flash Partition Layout

| Name | Type | Offset | Size |
|------|------|--------|------|
| nvs | data | 0x9000 | 20KB |
| otadata | data | 0xE000 | 8KB |
| app0 | app | 0x10000 | 1.9MB |
| app1 | app | 0x1F0000 | 1.9MB |
| spiffs | data | 0x3D0000 | 192KB |

## Protocol Support

- **Standard:** ASTM F3411-22 (Remote ID v2)
- **Backward compatible:** Protocol versions 0-2
- **Message types:** Basic ID, Location, System, Self ID, Operator ID, Authentication
- **BLE Service UUID:** 0xFFFA
