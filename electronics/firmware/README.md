# NL-1 Pioneer — Onboard Rocket Computer Firmware

Firmware for the **Seeed Studio XIAO ESP32-S3 Sense** flight computer on the NeoLabs NL-1 Pioneer. It records video and full telemetry to microSD, detects flight phases, and talks to the Mission Dashboard over a **second, independent BLE link**.

**This device never fires a motor, igniter, or any pyrotechnic.** Ignition remains exclusively with the Launch Controller.

## Hardware (from `connections.md` + schematic)

| Item | Detail |
|------|--------|
| MCU | Seeed XIAO ESP32-S3 Sense (8 MB PSRAM) |
| IMU | Adafruit LSM6DSO32 (U3) |
| Baro | Adafruit BMP580 (U2) |
| I2C | SDA **GPIO5 (D4)**, SCL **GPIO6 (D5)** @ 400 kHz |
| Camera | Onboard OV2640/OV3660 (Sense expansion) |
| microSD | Onboard SPI: CS GPIO21, MOSI GPIO9, MISO GPIO8, SCK GPIO7 |
| Power | Battery → SW1 → BAT+ / BAT− |
| Battery ADC | **Not present** on schematic — not measured |

Unused sensor pins (CS, SDO, INT1/2) are left open per `connections.md`.

## Build & flash

```bash
cd electronics/firmware
pio run -e xiao_esp32s3_sense
pio run -e xiao_esp32s3_sense -t upload
pio device monitor -b 115200
```

Requirements: PlatformIO, USB cable, board in download mode if needed (BOOT + RESET).

### Host unit tests

```bash
pio test -e native_tests
```

## Architecture

```
sensor task (100 Hz)  →  flight detect  →  mission FSM
                      →  telemetry queue →  storage task → microSD
camera task           →  JPEG frames    →  storage task → video.mjpg
BLE (NimBLE)          →  command queue  →  mission FSM
                      ←  status / telemetry notifies
```

Priorities: **sensors > storage > camera/mission > BLE live telemetry**.

Modules under `src/` / `include/`:

| Module | Role |
|--------|------|
| `time_manager` | Monotonic time + optional external sync |
| `mission_fsm` | Mission / recording state machine |
| `flight_detect` | Multi-criteria phase detection |
| `sensors` | LSM6DSO32 + BMP580 + MCU temp |
| `storage` | Single SD writer, session files |
| `camera_recorder` | Profiled JPEG capture with fallback |
| `ble_server` / `command_handler` | Protocol + idempotent commands |
| `event_log` | Async events.jsonl lines |

## BLE

- Device name: `NeoLabs Rocket Computer`
- Service UUID: `9c4e0001-6a2b-4c8d-9e1f-1d6c7a0b2000`  
  (Launch Controller uses `8f3a0001-…` — **separate**.)
- Command JSON (same spirit as Launch System): `{"cmd":"arm","sid":"…","seq":N}`

See [docs/BLE_PROTOCOL.md](docs/BLE_PROTOCOL.md).

## Recording timeline

```
IDLE → ARM → session + video + telemetry start
     → countdown (optional) + expected ignition time
     → sensor liftoff → ascent / coast / apogee / descent
     → multi-criteria LANDING → +30 s → flush & close
```

- BLE disconnect **never** stops recording or countdown.
- `STOP_RECORDING` / `RESET` rejected during flight.
- Hard safety cap: **10 minutes** max recording if landing never confirms.
- Post-landing: **30 seconds**.

## Documentation index

- [docs/HARDWARE.md](docs/HARDWARE.md)
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BLE_PROTOCOL.md](docs/BLE_PROTOCOL.md)
- [docs/STATE_MACHINE.md](docs/STATE_MACHINE.md)
- [docs/FLIGHT_DETECTION.md](docs/FLIGHT_DETECTION.md)
- [docs/CAMERA.md](docs/CAMERA.md)
- [docs/STORAGE.md](docs/STORAGE.md)
- [docs/DASHBOARD.md](docs/DASHBOARD.md)
- [docs/TESTING.md](docs/TESTING.md)
- [docs/ERROR_CODES.md](docs/ERROR_CODES.md)

## Dashboard

Mission Dashboard (`~/NeoLabs-Rockets/MissionDashboard`) was extended with:

- `rocket-ble-link.js` — independent GATT connection manager
- `rocket-computer.js` — UI + command mirroring
- Arm / countdown / abort on the launch console also notify the rocket computer when linked

## Safety

| Action | Rocket computer |
|--------|-----------------|
| Ignition / relay | **Never** |
| Pyro / servo deploy | **Never** |
| Video + telemetry | Yes |
| Flight phase log | Yes |
| Live BLE status | Yes (best-effort) |
