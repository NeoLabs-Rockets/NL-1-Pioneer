#include "mission_fsm.h"
#include "event_log.h"
#include "time_manager.h"
#include "config.h"

#include <string.h>
#include <stdio.h>

namespace MissionFsm {
namespace {

MissionState mission = MissionState::BOOT;
RecordingState recording = RecordingState::IDLE;
FlightPhase phase = FlightPhase::PRELAUNCH;

char owner[40] = "";
char abort_reason[64] = "";

int64_t expected_ign_mono_ms = -1;
int64_t actual_liftoff_mono_ms = -1;
int64_t countdown_start_mono_ms = -1;
uint32_t countdown_duration_ms = 0;
bool countdown_on = false;
bool lifted_off = false;

int64_t landed_at_mono_ms = -1;
int64_t recording_started_mono_ms = -1;
bool ble_connected = false;

// Request flags consumed by higher layers (storage/camera).
bool request_start_recording = false;
bool request_stop_recording = false;
char recording_start_reason[40] = "";

void transition(MissionState next, const char* cause) {
  if (next == mission) return;
  MissionState prev = mission;
  mission = next;
  EventLog::emitStateChange(prev, next, cause);
}

bool sidMatches(const char* sid) {
  if (!owner[0]) return true;
  if (!sid || !sid[0]) return false;
  return strcmp(owner, sid) == 0;
}

void setOwner(const char* sid) {
  if (sid && sid[0]) {
    strncpy(owner, sid, sizeof(owner) - 1);
    owner[sizeof(owner) - 1] = '\0';
  }
}

void clearCountdown(const char* reason) {
  (void)reason;
  countdown_on = false;
  // Keep expected_ign_mono_ms history for logs; clear active flag only.
}

}  // namespace

void begin() {
  mission = MissionState::BOOT;
  recording = RecordingState::IDLE;
  phase = FlightPhase::PRELAUNCH;
  owner[0] = '\0';
  abort_reason[0] = '\0';
  expected_ign_mono_ms = -1;
  actual_liftoff_mono_ms = -1;
  countdown_start_mono_ms = -1;
  countdown_duration_ms = 0;
  countdown_on = false;
  lifted_off = false;
  landed_at_mono_ms = -1;
  recording_started_mono_ms = -1;
  ble_connected = false;
  request_start_recording = false;
  request_stop_recording = false;
}

MissionState state() { return mission; }
RecordingState recordingState() { return recording; }
FlightPhase flightPhase() { return phase; }

void setBluetoothConnected(bool connected) {
  ble_connected = connected;
  if (connected) {
    if (mission == MissionState::IDLE) transition(MissionState::CONNECTED, "ble_connect");
    EventLog::emit(EventType::BLE_CONNECT, "connected");
  } else {
    EventLog::emit(EventType::BLE_DISCONNECT, "disconnected");
    // Never stop recording or cancel countdown on BLE loss.
    if (mission == MissionState::CONNECTED) transition(MissionState::IDLE, "ble_disconnect");
  }
}

void setFlightPhase(FlightPhase p, const char* cause) {
  if (p == phase) return;
  FlightPhase prev = phase;
  phase = p;
  char extra[96];
  snprintf(extra, sizeof(extra), "\"from\":\"%s\",\"to\":\"%s\",\"cause\":\"%s\"",
           flightPhaseName(prev), flightPhaseName(p), cause ? cause : "");
  EventLog::emit(EventType::FLIGHT_PHASE, "phase", extra);

  const uint64_t now = TimeManager::monoMs();
  switch (p) {
    case FlightPhase::LIFTOFF:
    case FlightPhase::POWERED_ASCENT:
      if (!lifted_off) {
        lifted_off = true;
        actual_liftoff_mono_ms = static_cast<int64_t>(now);
        EventLog::emit(EventType::LIFTOFF, "liftoff_detected", extra);
      }
      if (!missionPostLiftoff(mission) || mission == MissionState::IGNITION_EXPECTED
          || mission == MissionState::COUNTDOWN || mission == MissionState::RECORDING
          || mission == MissionState::ARMED || mission == MissionState::ABORTED) {
        // ABORTED after liftoff must not pretend flight never happened — move to ASCENT.
        transition(MissionState::ASCENT, cause);
      }
      break;
    case FlightPhase::COAST:
      if (missionInFlight(mission) || mission == MissionState::ASCENT)
        transition(MissionState::COAST, cause);
      break;
    case FlightPhase::APOGEE:
      if (missionInFlight(mission)) transition(MissionState::APOGEE, cause);
      EventLog::emit(EventType::APOGEE, "apogee_detected");
      break;
    case FlightPhase::DESCENT:
    case FlightPhase::IMPACT:
      if (missionInFlight(mission) || mission == MissionState::APOGEE)
        transition(MissionState::DESCENT, cause);
      break;
    case FlightPhase::LANDED:
      if (missionPostLiftoff(mission) || missionInFlight(mission) || mission == MissionState::DESCENT) {
        transition(MissionState::LANDED, cause);
        landed_at_mono_ms = static_cast<int64_t>(now);
        EventLog::emit(EventType::LANDING, "landing_confirmed");
        if (recording == RecordingState::ACTIVE) {
          recording = RecordingState::POST_LANDING;
        }
      }
      break;
    default:
      break;
  }
}

void setRecordingState(RecordingState rs) {
  recording = rs;
  if (rs == RecordingState::ACTIVE && recording_started_mono_ms < 0) {
    recording_started_mono_ms = static_cast<int64_t>(TimeManager::monoMs());
  }
}

void setError(const char* reason) {
  transition(MissionState::ERROR, reason ? reason : "error");
}

CmdResult cmdArm(const char* sid) {
  if (missionInFlight(mission) || mission == MissionState::LANDED) {
    return CmdResult::REJECTED;
  }
  if (mission == MissionState::ARMED || mission == MissionState::RECORDING
      || mission == MissionState::COUNTDOWN || mission == MissionState::IGNITION_EXPECTED) {
    // Idempotent ARM
    setOwner(sid);
    return CmdResult::IGNORED;
  }
  if (mission != MissionState::IDLE && mission != MissionState::CONNECTED
      && mission != MissionState::ABORTED) {
    return CmdResult::REJECTED;
  }
  setOwner(sid);
  transition(MissionState::ARMED, "cmd_arm");
  // Recording begins on arm (requirement).
  cmdStartRecording("arm");
  return CmdResult::ACK;
}

CmdResult cmdDisarm(const char* sid) {
  if (missionInFlight(mission) || mission == MissionState::LANDED) {
    return CmdResult::REJECTED;
  }
  if (!sidMatches(sid) && owner[0]) return CmdResult::REJECTED;
  if (mission == MissionState::IDLE || mission == MissionState::CONNECTED) {
    return CmdResult::IGNORED;
  }
  clearCountdown("disarm");
  expected_ign_mono_ms = -1;
  countdown_on = false;
  // Stop recording only if not in flight (already checked).
  if (recording == RecordingState::ACTIVE || recording == RecordingState::POST_LANDING
      || recording == RecordingState::PREPARING) {
    request_stop_recording = true;
    recording = RecordingState::STOPPING;
  }
  owner[0] = '\0';
  transition(ble_connected ? MissionState::CONNECTED : MissionState::IDLE, "cmd_disarm");
  return CmdResult::ACK;
}

CmdResult cmdPrepareRecording() {
  if (recording == RecordingState::ACTIVE || recording == RecordingState::PREPARING) {
    return CmdResult::IGNORED;
  }
  if (missionInFlight(mission)) return CmdResult::REJECTED;
  recording = RecordingState::PREPARING;
  return CmdResult::ACK;
}

CmdResult cmdStartRecording(const char* reason) {
  if (recording == RecordingState::ACTIVE || recording == RecordingState::POST_LANDING) {
    return CmdResult::IGNORED;  // idempotent — no second session
  }
  if (mission == MissionState::ERROR) return CmdResult::REJECTED;
  request_start_recording = true;
  strncpy(recording_start_reason, reason ? reason : "start", sizeof(recording_start_reason) - 1);
  recording_start_reason[sizeof(recording_start_reason) - 1] = '\0';
  if (mission == MissionState::ARMED || mission == MissionState::CONNECTED
      || mission == MissionState::IDLE || mission == MissionState::ABORTED) {
    if (mission != MissionState::COUNTDOWN && mission != MissionState::IGNITION_EXPECTED)
      transition(MissionState::RECORDING, reason ? reason : "start_recording");
  }
  recording = RecordingState::PREPARING;
  EventLog::emit(EventType::RECORDING_START, reason ? reason : "start");
  return CmdResult::ACK;
}

CmdResult cmdStopRecording(const char* reason, bool force) {
  if (!force && missionInFlight(mission)) return CmdResult::REJECTED;
  if (recording == RecordingState::IDLE || recording == RecordingState::STOPPED) {
    return CmdResult::IGNORED;
  }
  request_stop_recording = true;
  recording = RecordingState::STOPPING;
  EventLog::emit(EventType::RECORDING_STOP, reason ? reason : "stop");
  return CmdResult::ACK;
}

CmdResult cmdStartCountdown(uint32_t duration_ms, int64_t expected_ignition_mono_ms) {
  if (missionInFlight(mission) || mission == MissionState::LANDED) {
    return CmdResult::REJECTED;
  }
  if (mission != MissionState::ARMED && mission != MissionState::RECORDING
      && mission != MissionState::COUNTDOWN && mission != MissionState::IGNITION_EXPECTED) {
    // Allow countdown from armed/recording only (must arm first).
    if (mission == MissionState::IDLE || mission == MissionState::CONNECTED) {
      return CmdResult::REJECTED;
    }
  }
  if (countdown_on && expected_ign_mono_ms == expected_ignition_mono_ms) {
    return CmdResult::IGNORED;  // duplicate
  }
  countdown_on = true;
  countdown_duration_ms = duration_ms;
  countdown_start_mono_ms = static_cast<int64_t>(TimeManager::monoMs());
  expected_ign_mono_ms = expected_ignition_mono_ms >= 0
                             ? expected_ignition_mono_ms
                             : countdown_start_mono_ms + static_cast<int64_t>(duration_ms);
  transition(MissionState::COUNTDOWN, "countdown_start");

  // Recording must be running; for short countdowns start immediately.
  if (recording != RecordingState::ACTIVE && recording != RecordingState::POST_LANDING
      && recording != RecordingState::PREPARING) {
    cmdStartRecording(duration_ms < RECORDING_LEAD_BEFORE_IGN_MS ? "countdown_short" : "countdown");
  }
  return CmdResult::ACK;
}

CmdResult cmdUpdateCountdown(uint32_t remaining_ms, int64_t expected_ignition_mono_ms) {
  if (!countdown_on && mission != MissionState::COUNTDOWN && mission != MissionState::IGNITION_EXPECTED) {
    return CmdResult::REJECTED;
  }
  if (expected_ignition_mono_ms >= 0) expected_ign_mono_ms = expected_ignition_mono_ms;
  else if (remaining_ms > 0) {
    expected_ign_mono_ms = static_cast<int64_t>(TimeManager::monoMs()) + remaining_ms;
  }
  return CmdResult::ACK;
}

CmdResult cmdSetExpectedIgnition(int64_t expected_ignition_mono_ms) {
  if (expected_ignition_mono_ms < 0) return CmdResult::NACK;
  expected_ign_mono_ms = expected_ignition_mono_ms;
  return CmdResult::ACK;
}

CmdResult cmdCancelCountdown(const char* reason) {
  if (!countdown_on && mission != MissionState::COUNTDOWN && mission != MissionState::IGNITION_EXPECTED) {
    return CmdResult::IGNORED;
  }
  if (missionInFlight(mission)) {
    // After liftoff, cancel is advisory only.
    EventLog::emit(EventType::WARNING, "countdown_cancel_ignored_in_flight", reason);
    return CmdResult::IGNORED;
  }
  clearCountdown(reason);
  countdown_on = false;
  if (mission == MissionState::COUNTDOWN || mission == MissionState::IGNITION_EXPECTED) {
    transition(recording == RecordingState::ACTIVE ? MissionState::RECORDING : MissionState::ARMED,
               reason ? reason : "countdown_cancel");
  }
  return CmdResult::ACK;
}

CmdResult cmdAbort(const char* reason) {
  strncpy(abort_reason, reason ? reason : "abort", sizeof(abort_reason) - 1);
  abort_reason[sizeof(abort_reason) - 1] = '\0';
  EventLog::emit(EventType::ABORT, abort_reason);

  // After liftoff: do not treat as "never launched".
  if (lifted_off || missionInFlight(mission) || mission == MissionState::LANDED) {
    countdown_on = false;
    // Advisory only — flight recording continues.
    return CmdResult::ACK;
  }

  countdown_on = false;
  // Keep recording files consistent; do not destroy telemetry.
  // Move to ABORTED while recording may continue briefly until operator disarms/stops.
  if (mission == MissionState::BOOT || mission == MissionState::SELF_TEST
      || mission == MissionState::IDLE || mission == MissionState::CONNECTED) {
    // No-op mission-wise beyond log
    return CmdResult::ACK;
  }
  transition(MissionState::ABORTED, abort_reason);
  return CmdResult::ACK;
}

CmdResult cmdReset(bool force) {
  if (!force && (missionInFlight(mission) || recording == RecordingState::POST_LANDING
                 || recording == RecordingState::ACTIVE)) {
    return CmdResult::REJECTED;
  }
  if (recording == RecordingState::ACTIVE || recording == RecordingState::POST_LANDING
      || recording == RecordingState::PREPARING || recording == RecordingState::STOPPING) {
    request_stop_recording = true;
    recording = RecordingState::STOPPING;
  }
  owner[0] = '\0';
  abort_reason[0] = '\0';
  expected_ign_mono_ms = -1;
  actual_liftoff_mono_ms = -1;
  countdown_start_mono_ms = -1;
  countdown_duration_ms = 0;
  countdown_on = false;
  lifted_off = false;
  landed_at_mono_ms = -1;
  phase = FlightPhase::PRELAUNCH;
  transition(ble_connected ? MissionState::CONNECTED : MissionState::IDLE, "reset");
  return CmdResult::ACK;
}

void ensureRecordingLead(uint64_t now_ms) {
  if (expected_ign_mono_ms < 0) return;
  if (recording == RecordingState::ACTIVE || recording == RecordingState::PREPARING
      || recording == RecordingState::POST_LANDING) {
    return;
  }
  const int64_t remaining = expected_ign_mono_ms - static_cast<int64_t>(now_ms);
  if (remaining >= 0 && remaining <= static_cast<int64_t>(RECORDING_LEAD_BEFORE_IGN_MS)) {
    cmdStartRecording("lead_time");
  }
}

void tick(uint64_t now_ms) {
  ensureRecordingLead(now_ms);

  if (countdown_on && expected_ign_mono_ms >= 0) {
    if (static_cast<int64_t>(now_ms) >= expected_ign_mono_ms
        && !lifted_off
        && (mission == MissionState::COUNTDOWN || mission == MissionState::RECORDING
            || mission == MissionState::ARMED)) {
      transition(MissionState::IGNITION_EXPECTED, "ignition_time_reached");
    }
  }

  // Post-landing recording window
  if (recording == RecordingState::POST_LANDING && landed_at_mono_ms >= 0) {
    if (now_ms >= static_cast<uint64_t>(landed_at_mono_ms) + POST_LANDING_RECORD_MS) {
      request_stop_recording = true;
      recording = RecordingState::STOPPING;
      EventLog::emit(EventType::RECORDING_STOP, "post_landing_timeout");
    }
  }

  // Hard safety timeout for any active recording
  if ((recording == RecordingState::ACTIVE || recording == RecordingState::POST_LANDING)
      && recording_started_mono_ms >= 0) {
    if (now_ms >= static_cast<uint64_t>(recording_started_mono_ms) + MAX_MISSION_RECORD_MS) {
      request_stop_recording = true;
      recording = RecordingState::STOPPING;
      EventLog::emit(EventType::WARNING, "max_mission_record_timeout");
    }
  }
}

bool recordingActive() {
  return recording == RecordingState::ACTIVE || recording == RecordingState::POST_LANDING
      || recording == RecordingState::PREPARING;
}

bool countdownActive() { return countdown_on; }
bool hasLiftedOff() { return lifted_off; }
int64_t expectedIgnitionMonoMs() { return expected_ign_mono_ms; }
int64_t actualLiftoffMonoMs() { return actual_liftoff_mono_ms; }
int64_t countdownStartMonoMs() { return countdown_start_mono_ms; }
uint32_t countdownDurationMs() { return countdown_duration_ms; }

int32_t countdownRemainingMs(uint64_t now_ms) {
  if (!countdown_on || expected_ign_mono_ms < 0) return -1;
  int64_t left = expected_ign_mono_ms - static_cast<int64_t>(now_ms);
  if (left < 0) return 0;
  if (left > INT32_MAX) return INT32_MAX;
  return static_cast<int32_t>(left);
}

const char* lastAbortReason() { return abort_reason; }
const char* ownerSid() { return owner; }

// Internal flags for main/storage integration
bool consumeStartRecording(char* reason_out, size_t n) {
  if (!request_start_recording) return false;
  request_start_recording = false;
  if (reason_out && n) {
    strncpy(reason_out, recording_start_reason, n - 1);
    reason_out[n - 1] = '\0';
  }
  return true;
}

bool consumeStopRecording() {
  if (!request_stop_recording) return false;
  request_stop_recording = false;
  return true;
}

void markRecordingActive() {
  recording = RecordingState::ACTIVE;
  if (recording_started_mono_ms < 0)
    recording_started_mono_ms = static_cast<int64_t>(TimeManager::monoMs());
  if (mission == MissionState::ARMED || mission == MissionState::IDLE
      || mission == MissionState::CONNECTED) {
    transition(MissionState::RECORDING, "recording_active");
  }
}

void markRecordingStopped() {
  recording = RecordingState::STOPPED;
  recording_started_mono_ms = -1;
  TimeManager::endSession();
}

void markRecordingFailed() {
  recording = RecordingState::FAILED;
}

void advanceFromBoot() {
  if (mission == MissionState::BOOT) transition(MissionState::SELF_TEST, "boot");
}

void advanceSelfTestOk() {
  if (mission == MissionState::SELF_TEST || mission == MissionState::BOOT) {
    transition(MissionState::IDLE, "self_test_ok");
  }
}

}  // namespace MissionFsm
