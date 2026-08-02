#pragma once
/**
 * Shared types for NL-1 rocket computer: states, telemetry sample, events.
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// ── Mission state (primary high-level FSM) ──────────────────────────────────
enum class MissionState : uint8_t {
  BOOT = 0,
  SELF_TEST,
  IDLE,
  CONNECTED,
  ARMED,
  RECORDING,
  COUNTDOWN,
  IGNITION_EXPECTED,
  ASCENT,
  COAST,
  APOGEE,
  DESCENT,
  LANDED,
  ABORTED,
  ERROR,
  COUNT
};

inline const char* missionStateName(MissionState s) {
  switch (s) {
    case MissionState::BOOT: return "BOOT";
    case MissionState::SELF_TEST: return "SELF_TEST";
    case MissionState::IDLE: return "IDLE";
    case MissionState::CONNECTED: return "CONNECTED";
    case MissionState::ARMED: return "ARMED";
    case MissionState::RECORDING: return "RECORDING";
    case MissionState::COUNTDOWN: return "COUNTDOWN";
    case MissionState::IGNITION_EXPECTED: return "IGNITION_EXPECTED";
    case MissionState::ASCENT: return "ASCENT";
    case MissionState::COAST: return "COAST";
    case MissionState::APOGEE: return "APOGEE";
    case MissionState::DESCENT: return "DESCENT";
    case MissionState::LANDED: return "LANDED";
    case MissionState::ABORTED: return "ABORTED";
    case MissionState::ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

// True once actual flight has been detected (liftoff and beyond).
inline bool missionInFlight(MissionState s) {
  return s == MissionState::ASCENT || s == MissionState::COAST
      || s == MissionState::APOGEE || s == MissionState::DESCENT;
}

inline bool missionPostLiftoff(MissionState s) {
  return missionInFlight(s) || s == MissionState::LANDED;
}

// ── Orthogonal subsystem states ─────────────────────────────────────────────
enum class RecordingState : uint8_t {
  IDLE = 0,
  PREPARING,
  ACTIVE,
  POST_LANDING,
  STOPPING,
  STOPPED,
  FAILED
};

enum class StorageState : uint8_t {
  UNKNOWN = 0,
  MISSING,
  MOUNTED,
  FULL,
  ERROR,
  WRITING
};

enum class BluetoothState : uint8_t {
  OFF = 0,
  ADVERTISING,
  CONNECTED,
  STALE
};

enum class CameraState : uint8_t {
  UNINIT = 0,
  READY,
  RECORDING,
  FAILED,
  FALLBACK
};

enum class SensorHealth : uint8_t {
  OK = 0,
  DEGRADED,
  FAILED,
  MISSING
};

// ── Flight phase (sensor-derived, independent of mission FSM labels) ────────
enum class FlightPhase : uint8_t {
  PRELAUNCH = 0,
  LIFTOFF,
  POWERED_ASCENT,
  COAST,
  APOGEE,
  DESCENT,
  IMPACT,
  LANDED,
  UNKNOWN
};

inline const char* flightPhaseName(FlightPhase p) {
  switch (p) {
    case FlightPhase::PRELAUNCH: return "PRELAUNCH";
    case FlightPhase::LIFTOFF: return "LIFTOFF";
    case FlightPhase::POWERED_ASCENT: return "POWERED_ASCENT";
    case FlightPhase::COAST: return "COAST";
    case FlightPhase::APOGEE: return "APOGEE";
    case FlightPhase::DESCENT: return "DESCENT";
    case FlightPhase::IMPACT: return "IMPACT";
    case FlightPhase::LANDED: return "LANDED";
    default: return "UNKNOWN";
  }
}

// ── Error classes ───────────────────────────────────────────────────────────
enum class ErrorClass : uint8_t {
  INFO = 0,
  WARNING,
  RECOVERABLE,
  CRITICAL
};

// ── Temperature channels (must not be confused) ─────────────────────────────
enum class TempSensorId : uint8_t {
  BMP580 = 0,     // barometer die temperature
  LSM6DSO32 = 1,  // IMU die temperature
  MCU_INTERNAL = 2,
  COUNT
};

inline const char* tempSensorIdName(TempSensorId id) {
  switch (id) {
    case TempSensorId::BMP580: return "BMP580";
    case TempSensorId::LSM6DSO32: return "LSM6DSO32";
    case TempSensorId::MCU_INTERNAL: return "MCU_INTERNAL";
    default: return "UNKNOWN";
  }
}

struct TemperatureSample {
  TempSensorId id;
  float celsius;
  bool valid;
  uint64_t mono_us;  // monotonic us since boot
};

// ── Time stamp bundle ───────────────────────────────────────────────────────
struct TimeStamp {
  uint64_t mono_us;          // monotonic since boot (always valid)
  uint64_t session_us;       // monotonic since flight session start (0 if none)
  int64_t  sync_unix_ms;     // synchronized external time, 0 if unsynced
  uint8_t  time_quality;     // 0=none, 1=boot-only, 2=sync-coarse, 3=sync-rtt
};

// ── Sensor sample (fixed layout for binary telemetry log) ───────────────────
// Little-endian packed record written to telemetry.bin
#pragma pack(push, 1)
struct TelemetryRecord {
  uint32_t magic;            // 0x4E4C3154 'NL1T'
  uint16_t version;          // 1
  uint16_t size;             // sizeof(TelemetryRecord)
  uint64_t mono_us;
  uint64_t session_us;
  int64_t  sync_unix_ms;

  float ax, ay, az;          // m/s²
  float gx, gy, gz;          // deg/s
  float accel_mag;           // m/s²
  float pressure_pa;
  float altitude_m;
  float vert_vel_ms;
  float temp_bmp_c;
  float temp_imu_c;
  float temp_mcu_c;

  uint8_t mission_state;
  uint8_t recording_state;
  uint8_t flight_phase;
  uint8_t flags;             // bit0 imu_ok, bit1 baro_ok, bit2 cam_ok, bit3 sd_ok

  uint32_t frame_count;
  uint16_t actual_fps_x10;
  uint16_t dropped_frames;
  uint32_t free_sd_kb;
  uint32_t seq;
  uint16_t crc16;            // CRC over all preceding bytes
};
#pragma pack(pop)

static constexpr uint32_t TELEM_MAGIC = 0x4E4C3154u;

// ── Live in-memory snapshot for BLE / mission logic ─────────────────────────
struct SensorSnapshot {
  TimeStamp ts;
  float ax, ay, az;
  float gx, gy, gz;
  float accel_mag;
  float pressure_pa;
  float altitude_m;
  float altitude_agl_m;
  float vert_vel_ms;
  TemperatureSample temps[static_cast<int>(TempSensorId::COUNT)];
  SensorHealth imu_health;
  SensorHealth baro_health;
  bool valid;
};

struct SystemSnapshot {
  MissionState mission;
  RecordingState recording;
  StorageState storage;
  BluetoothState bluetooth;
  CameraState camera;
  FlightPhase phase;
  uint32_t uptime_ms;
  int32_t countdown_left_ms;       // -1 if not in countdown
  int64_t expected_ignition_mono_ms;
  int64_t actual_liftoff_mono_ms;
  uint32_t frames_written;
  uint32_t frames_dropped;
  float actual_fps;
  uint64_t free_sd_bytes;
  uint64_t session_bytes;
  uint8_t camera_profile;
  char last_error[48];
  char last_warning[48];
  bool ground_test;
};

// ── Events for events.jsonl ─────────────────────────────────────────────────
enum class EventType : uint8_t {
  BOOT = 0,
  STATE_CHANGE,
  COMMAND,
  COMMAND_RESULT,
  BLE_CONNECT,
  BLE_DISCONNECT,
  SENSOR_FAULT,
  CAMERA_FAULT,
  STORAGE_FAULT,
  FLIGHT_PHASE,
  LIFTOFF,
  APOGEE,
  LANDING,
  ABORT,
  RECORDING_START,
  RECORDING_STOP,
  TIME_SYNC,
  WARNING,
  ERROR,
  RESET_REASON,
  HEARTBEAT,
  GROUND_TEST
};

// ── BLE command IDs (string names match Launch-System style JSON "cmd") ─────
// Names aligned with Mission Dashboard conventions where possible.
namespace Cmd {
  constexpr const char* GET_STATUS = "status";
  constexpr const char* GET_CAPABILITIES = "capabilities";
  constexpr const char* SYNC_TIME = "sync_time";
  constexpr const char* PREPARE_RECORDING = "prepare_recording";
  constexpr const char* ARM = "arm";
  constexpr const char* DISARM = "disarm";
  constexpr const char* START_RECORDING = "start_recording";
  constexpr const char* STOP_RECORDING = "stop_recording";
  constexpr const char* START_COUNTDOWN = "countdown_start";
  constexpr const char* UPDATE_COUNTDOWN = "countdown_update";
  constexpr const char* SET_EXPECTED_IGNITION = "set_expected_ignition";
  constexpr const char* CANCEL_COUNTDOWN = "countdown_cancel";
  constexpr const char* ABORT = "abort";
  constexpr const char* RESET = "reset";
  constexpr const char* PING = "ping";
  constexpr const char* GROUND_TEST = "ground_test";
}

// Command allow / deny results
enum class CmdResult : uint8_t {
  ACK = 0,
  NACK,
  IGNORED,       // idempotent duplicate or already-satisfied
  REJECTED       // illegal in current state
};
