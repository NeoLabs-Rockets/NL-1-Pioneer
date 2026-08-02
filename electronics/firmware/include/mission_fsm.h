#pragma once
#include "types.h"

/**
 * Mission finite-state machine.
 *
 * Abort semantics by state:
 *   BOOT/SELF_TEST/IDLE/CONNECTED → log, stay/return IDLE
 *   ARMED/RECORDING                → stop countdown refs, keep recording, → ABORTED
 *   COUNTDOWN/IGNITION_EXPECTED    → cancel countdown, keep recording, → ABORTED
 *   ASCENT/COAST/APOGEE/DESCENT    → log abort as advisory only; do NOT rewrite
 *                                    flight as "never launched"; remain in flight states
 *   LANDED                         → log, ignore mission effect
 *   ABORTED/ERROR                  → log, ignore
 *
 * Recording is never stopped by BLE disconnect.
 * STOP_RECORDING rejected while missionInFlight().
 * RESET rejected while missionInFlight() or active post-landing window.
 */
namespace MissionFsm {

void begin();

MissionState state();
RecordingState recordingState();
FlightPhase flightPhase();

void setBluetoothConnected(bool connected);
void setFlightPhase(FlightPhase phase, const char* cause);
void setRecordingState(RecordingState rs);
void setError(const char* reason);

// Command handlers — return CmdResult; side effects update FSM.
CmdResult cmdArm(const char* sid);
CmdResult cmdDisarm(const char* sid);
CmdResult cmdPrepareRecording();
CmdResult cmdStartRecording(const char* reason);
CmdResult cmdStopRecording(const char* reason, bool force = false);
CmdResult cmdStartCountdown(uint32_t duration_ms, int64_t expected_ignition_mono_ms);
CmdResult cmdUpdateCountdown(uint32_t remaining_ms, int64_t expected_ignition_mono_ms);
CmdResult cmdSetExpectedIgnition(int64_t expected_ignition_mono_ms);
CmdResult cmdCancelCountdown(const char* reason);
CmdResult cmdAbort(const char* reason);
CmdResult cmdReset(bool force = false);

// Periodic tick (mission task): handles auto transitions (ignition expected,
// post-landing stop, max mission timeout).
void tick(uint64_t now_ms);

// Timing accessors
bool recordingActive();
bool countdownActive();
bool hasLiftedOff();
int64_t expectedIgnitionMonoMs();
int64_t actualLiftoffMonoMs();
int64_t countdownStartMonoMs();
uint32_t countdownDurationMs();
int32_t countdownRemainingMs(uint64_t now_ms);
const char* lastAbortReason();
const char* ownerSid();

// Ensure recording starts if within lead time of expected ignition.
void ensureRecordingLead(uint64_t now_ms);

}  // namespace MissionFsm
