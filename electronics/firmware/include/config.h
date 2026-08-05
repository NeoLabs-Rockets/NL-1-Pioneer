#pragma once
/**
 * Central configuration for NL-1 onboard rocket computer.
 * Thresholds, rates, camera profiles, and safety timeouts live here.
 */

#include <stddef.h>
#include <stdint.h>
#include "board_pins.h"

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "1.0.0"
#endif

#define PROTOCOL_VERSION_MAJOR 1
#define PROTOCOL_VERSION_MINOR 0

// ── Task priorities (higher = more important) ───────────────────────────────
// Priority order from requirements:
//   1 sensors  2 storage  3 video  4 state machine  5 BLE telemetry
static constexpr uint8_t PRIO_SENSOR   = 5;
static constexpr uint8_t PRIO_STORAGE  = 4;
static constexpr uint8_t PRIO_CAMERA   = 3;
static constexpr uint8_t PRIO_MISSION  = 3;
static constexpr uint8_t PRIO_BLE      = 2;
static constexpr uint8_t PRIO_WATCHDOG = 1;

// ── Sampling rates ──────────────────────────────────────────────────────────
static constexpr uint32_t SENSOR_HZ_IMU       = 100;  // LSM6DSO32 sample target
static constexpr uint32_t SENSOR_HZ_BARO      = 50;   // BMP580 sample target
static constexpr uint32_t SENSOR_HZ_TEMP_MCU  = 1;
static constexpr uint32_t TELEMETRY_LOG_HZ    = 50;   // written to SD
static constexpr uint32_t BLE_TELEMETRY_HZ    = 5;    // live stream (degrades first)
static constexpr uint32_t BLE_TELEMETRY_HZ_MIN = 1;

// ── Camera profiles (highest stable first; runtime may fall back) ───────────
// "Max" is not the nominal peak resolution — it is the highest stable combo of
// resolution × fps × JPEG quality under concurrent sensor+SD+BLE load.
// Default primary profile: SVGA 800×600 @ 12 fps, JPEG quality 12, 2 FB in PSRAM.
// Expected data rate ≈ 0.4–0.9 MB/s depending on scene complexity.
enum class CameraProfileId : uint8_t {
  Primary_SVGA_12 = 0,
  Fallback_VGA_10 = 1,
  Low_QVGA_10     = 2,
  Count
};

// frame sizes match esp32-camera framesize_t
#define CAM_FRAMESIZE_SVGA  8   // 800x600
#define CAM_FRAMESIZE_VGA   6   // 640x480
#define CAM_FRAMESIZE_QVGA  5   // 320x240

// ── Recording / mission timeouts ────────────────────────────────────────────
// Model rocket motor burn + coast for NL-1 class motors is typically < 30 s.
// Conservative max flight window before force-stop if landing never confirms.
static constexpr uint32_t POST_LANDING_RECORD_MS     = 30000;   // +30 s after LANDED
static constexpr uint32_t MAX_MISSION_RECORD_MS      = 600000;  // 10 min hard safety cap
static constexpr uint32_t RECORDING_LEAD_BEFORE_IGN_MS = 30000;  // must be recording ≥30 s before ign
static constexpr uint32_t SD_FLUSH_INTERVAL_MS        = 1000;
static constexpr uint32_t SD_MIN_FREE_BYTES           = 32UL * 1024UL * 1024UL;  // refuse new session if <32 MB

// ── Flight phase detection (multi-criteria, hysteresis) ─────────────────────
// Accelerations in m/s², rates in deg/s, altitudes in m, velocities in m/s.
struct FlightDetectConfig {
  float liftoffAccelMagMs2;
  float liftoffAxialDeltaMs2;
  uint32_t liftoffMinMs;
  float burnoutAccelMs2;
  uint32_t burnoutMinMs;
  float apogeeVertVelMs;
  float apogeeAltWindowM;
  uint32_t apogeeMinMs;
  float descentVertVelMs;
  float impactAccelMagMs2;
  uint32_t impactWindowMs;
  float landedAccelBandMs2;
  float landedGyroMaxDps;
  float landedAltStdM;
  float landedVertVelMs;
  uint32_t landedConfirmMs;
  float minFlightAltGainM;
};

static constexpr FlightDetectConfig kFlightCfg = {
  /*liftoffAccelMagMs2*/   18.0f,
  /*liftoffAxialDeltaMs2*/ 12.0f,
  /*liftoffMinMs*/         80,
  /*burnoutAccelMs2*/      14.0f,
  /*burnoutMinMs*/         150,
  /*apogeeVertVelMs*/      2.5f,
  /*apogeeAltWindowM*/     3.0f,
  /*apogeeMinMs*/          200,
  /*descentVertVelMs*/     -2.0f,
  /*impactAccelMagMs2*/    25.0f,
  /*impactWindowMs*/       50,
  /*landedAccelBandMs2*/   2.5f,
  /*landedGyroMaxDps*/     15.0f,
  /*landedAltStdM*/        1.5f,
  /*landedVertVelMs*/      1.0f,
  /*landedConfirmMs*/      2500,
  /*minFlightAltGainM*/    5.0f,
};

// ── BLE ─────────────────────────────────────────────────────────────────────
static constexpr char BLE_DEVICE_NAME[] = "NeoLabs Rocket Computer";
// Distinct from Launch Controller service 8f3a0001-...
static constexpr char BLE_SERVICE_UUID[]   = "9c4e0001-6a2b-4c8d-9e1f-1d6c7a0b2000";
static constexpr char BLE_COMMAND_UUID[]   = "9c4e0002-6a2b-4c8d-9e1f-1d6c7a0b2000";
static constexpr char BLE_STATUS_UUID[]    = "9c4e0003-6a2b-4c8d-9e1f-1d6c7a0b2000";
static constexpr char BLE_TELEMETRY_UUID[] = "9c4e0004-6a2b-4c8d-9e1f-1d6c7a0b2000";
static constexpr char BLE_EVENT_UUID[]     = "9c4e0005-6a2b-4c8d-9e1f-1d6c7a0b2000";
// Binary file-download data channel (chunked flight export).
static constexpr char BLE_FILE_DATA_UUID[] = "9c4e0006-6a2b-4c8d-9e1f-1d6c7a0b2000";
// BLE OTA (same shape as Launch Controller 8f3a0004–0006).
static constexpr char BLE_OTA_CONTROL_UUID[] = "9c4e0007-6a2b-4c8d-9e1f-1d6c7a0b2000";
static constexpr char BLE_OTA_DATA_UUID[]    = "9c4e0008-6a2b-4c8d-9e1f-1d6c7a0b2000";
static constexpr char BLE_OTA_STATUS_UUID[]  = "9c4e0009-6a2b-4c8d-9e1f-1d6c7a0b2000";

static constexpr uint16_t BLE_MTU_TARGET = 185;
static constexpr size_t BLE_MSG_ID_CACHE = 32;
static constexpr uint32_t BLE_CMD_QUEUE_LEN = 16;
// File chunk payload budget after ATT headers (~3) and our 10-byte header.
static constexpr size_t BLE_FILE_CHUNK_MAX = 160;

// ── Storage layout ──────────────────────────────────────────────────────────
// Arduino-esp32 SD VFS mounts the card at "/sd" and *prepends* that mount point
// to every path passed to SD.open/mkdir/exists. Paths here must be card-root
// relative (e.g. "/flights"), NOT "/sd/flights" — otherwise you get "/sd/sd/...".
static constexpr char SD_MOUNT[] = "/sd";  // VFS mount point (informational)
static constexpr char FLIGHTS_DIR[] = "/flights";
static constexpr size_t TELEMETRY_RING_SAMPLES = 256;
static constexpr size_t EVENT_QUEUE_LEN = 64;
static constexpr size_t FRAME_QUEUE_LEN = 4;

// ── Watchdog ────────────────────────────────────────────────────────────────
static constexpr uint32_t WDT_TIMEOUT_S = 10;
