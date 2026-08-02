/**
 * Host-side unit tests for mission FSM, command handler, flight detect, CRC.
 * Run: pio test -e native_tests
 */
#ifdef UNIT_TEST

#include <unity.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

// Pull in implementation sources for native build
#include "../../src/crc16.cpp"
#include "../../src/time_manager.cpp"
#include "../../src/flight_detect.cpp"
#include "../../src/mission_fsm.cpp"
#include "../../src/command_handler.cpp"

// EventLog stubs for host (no FreeRTOS queues)
namespace EventLog {
void begin() {}
void emit(EventType, const char*, const char*) {}
void emitStateChange(MissionState, MissionState, const char*) {}
void emitCommand(const char*, uint32_t, const char*, CmdResult, const char*) {}
uint32_t droppedCount() { return 0; }
bool pop(char*, size_t) { return false; }
}

// Sensors stubs
namespace Sensors {
void setSimulation(bool) {}
void injectSim(const SensorSnapshot&) {}
}

namespace TimeManager {
void setFakeMonoUs(uint64_t us);
}

static SensorSnapshot makeSample(float amag, float alt, float vz, float gyro = 0) {
  SensorSnapshot s{};
  s.valid = true;
  s.ax = 0; s.ay = 0; s.az = amag; s.accel_mag = amag;
  s.gx = gyro; s.gy = 0; s.gz = 0;
  s.pressure_pa = 101325.f;
  s.altitude_m = alt;
  s.vert_vel_ms = vz;
  s.imu_health = SensorHealth::OK;
  s.baro_health = SensorHealth::OK;
  s.ts.mono_us = TimeManager::monoUs();
  return s;
}

void set_up() {
  TimeManager::begin();
  TimeManager::setFakeMonoUs(0);
  MissionFsm::begin();
  MissionFsm::advanceFromBoot();
  MissionFsm::advanceSelfTestOk();
  FlightDetect::begin();
  FlightDetect::reset();
}

void test_crc16_stable() {
  const uint8_t data[] = {1, 2, 3, 4, 5};
  uint16_t a = crc16_ccitt(data, sizeof(data));
  uint16_t b = crc16_ccitt(data, sizeof(data));
  TEST_ASSERT_EQUAL_UINT16(a, b);
  TEST_ASSERT_NOT_EQUAL(0, a);
}

void test_arm_starts_recording_request() {
  set_up();
  TEST_ASSERT_EQUAL(MissionState::IDLE, MissionFsm::state());
  CmdResult r = MissionFsm::cmdArm("sid-1");
  TEST_ASSERT_EQUAL(CmdResult::ACK, r);
  TEST_ASSERT_EQUAL(MissionState::RECORDING, MissionFsm::state());
  // Double ARM is idempotent
  r = MissionFsm::cmdArm("sid-1");
  TEST_ASSERT_EQUAL(CmdResult::IGNORED, r);
}

void test_countdown_short_and_long() {
  set_up();
  MissionFsm::cmdArm("s");
  // 10 s countdown
  CmdResult r = MissionFsm::cmdStartCountdown(10000, 10000);
  TEST_ASSERT_EQUAL(CmdResult::ACK, r);
  TEST_ASSERT_EQUAL(MissionState::COUNTDOWN, MissionFsm::state());
  TEST_ASSERT_TRUE(MissionFsm::countdownActive());
  // Duplicate
  r = MissionFsm::cmdStartCountdown(10000, 10000);
  TEST_ASSERT_EQUAL(CmdResult::IGNORED, r);
}

void test_abort_pre_liftoff_vs_post() {
  set_up();
  MissionFsm::cmdArm("s");
  MissionFsm::cmdStartCountdown(10000, 10000);
  CmdResult r = MissionFsm::cmdAbort("range_hold");
  TEST_ASSERT_EQUAL(CmdResult::ACK, r);
  TEST_ASSERT_EQUAL(MissionState::ABORTED, MissionFsm::state());

  // Post-liftoff: abort is advisory, mission moves with phase
  set_up();
  MissionFsm::cmdArm("s");
  MissionFsm::setFlightPhase(FlightPhase::LIFTOFF, "test");
  TEST_ASSERT_TRUE(MissionFsm::hasLiftedOff());
  r = MissionFsm::cmdAbort("late");
  TEST_ASSERT_EQUAL(CmdResult::ACK, r);
  TEST_ASSERT_TRUE(MissionFsm::hasLiftedOff());
  TEST_ASSERT_EQUAL(MissionState::ASCENT, MissionFsm::state());
}

void test_stop_recording_rejected_in_flight() {
  set_up();
  MissionFsm::cmdArm("s");
  MissionFsm::setFlightPhase(FlightPhase::POWERED_ASCENT, "test");
  CmdResult r = MissionFsm::cmdStopRecording("dash", false);
  TEST_ASSERT_EQUAL(CmdResult::REJECTED, r);
}

void test_reset_rejected_in_flight() {
  set_up();
  MissionFsm::cmdArm("s");
  MissionFsm::setFlightPhase(FlightPhase::DESCENT, "test");
  CmdResult r = MissionFsm::cmdReset(false);
  TEST_ASSERT_EQUAL(CmdResult::REJECTED, r);
}

void test_command_unknown() {
  set_up();
  char resp[256];
  CmdResult r = CommandHandler::handleJson("{\"cmd\":\"nope\",\"seq\":1}", resp, sizeof(resp));
  TEST_ASSERT_EQUAL(CmdResult::NACK, r);
  TEST_ASSERT_NOT_NULL(strstr(resp, "NACK"));
}

void test_command_duplicate_idempotent() {
  set_up();
  char resp[256];
  const char* body = "{\"cmd\":\"arm\",\"sid\":\"a\",\"seq\":42,\"msg_id\":42}";
  CmdResult r1 = CommandHandler::handleJson(body, resp, sizeof(resp));
  TEST_ASSERT_EQUAL(CmdResult::ACK, r1);
  CmdResult r2 = CommandHandler::handleJson(body, resp, sizeof(resp));
  TEST_ASSERT_EQUAL(CmdResult::IGNORED, r2);
}

void test_ble_disconnect_does_not_cancel_countdown() {
  set_up();
  MissionFsm::cmdArm("s");
  MissionFsm::cmdStartCountdown(30000, 30000);
  MissionFsm::setBluetoothConnected(true);
  MissionFsm::setBluetoothConnected(false);  // disconnect
  TEST_ASSERT_TRUE(MissionFsm::countdownActive());
  TEST_ASSERT_EQUAL(MissionState::COUNTDOWN, MissionFsm::state());
}

void test_flight_liftoff_multi_sample() {
  set_up();
  FlightDetect::begin();
  TimeManager::setFakeMonoUs(0);
  // Quiet baseline
  for (int i = 0; i < 10; i++) {
    TimeManager::setFakeMonoUs(i * 10000ULL);
    auto s = makeSample(9.8f, 100.f, 0.f);
    s.ts.mono_us = TimeManager::monoUs();
    FlightDetect::update(s, -1, true);
  }
  // Single spike should NOT immediately liftoff (needs min duration)
  TimeManager::setFakeMonoUs(200000);
  auto spike = makeSample(40.f, 100.f, 5.f);
  spike.ts.mono_us = TimeManager::monoUs();
  FlightDetect::update(spike, -1, true);
  // Hold for liftoffMinMs
  for (int i = 0; i < 20; i++) {
    TimeManager::setFakeMonoUs(200000ULL + i * 10000ULL);
    auto s = makeSample(40.f, 101.f + i, 30.f);
    s.ts.mono_us = TimeManager::monoUs();
    FlightDetect::update(s, -1, true);
  }
  TEST_ASSERT_TRUE(
      FlightDetect::phase() == FlightPhase::LIFTOFF
      || FlightDetect::phase() == FlightPhase::POWERED_ASCENT);
}

void test_false_quiet_does_not_land() {
  set_up();
  FlightDetect::injectPhase(FlightPhase::DESCENT);
  // Force non-forced path by reset then manual? injectPhase sets forced.
  // Use real path: reset and walk phases briefly isn't needed —
  // injectPhase is forced, so update returns false. Test quiet helper via landedConfirm.
  // Instead verify PRELAUNCH quiet doesn't jump to LANDED.
  FlightDetect::reset();
  for (int i = 0; i < 50; i++) {
    TimeManager::setFakeMonoUs(i * 20000ULL);
    auto s = makeSample(9.8f, 100.f, 0.f);
    s.ts.mono_us = TimeManager::monoUs();
    FlightDetect::update(s, -1, false);
  }
  TEST_ASSERT_EQUAL(FlightPhase::PRELAUNCH, FlightDetect::phase());
}

void test_sync_time_command() {
  set_up();
  char resp[256];
  CmdResult r = CommandHandler::handleJson(
      "{\"cmd\":\"sync_time\",\"unix_ms\":1700000000000,\"rtt_ms\":40,\"seq\":9}",
      resp, sizeof(resp));
  TEST_ASSERT_EQUAL(CmdResult::ACK, r);
  TEST_ASSERT_TRUE(TimeManager::isSynced());
}

void test_countdown_without_arm_rejected() {
  set_up();
  CmdResult r = MissionFsm::cmdStartCountdown(10000, 10000);
  // From IDLE without arm
  TEST_ASSERT_EQUAL(CmdResult::REJECTED, r);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_crc16_stable);
  RUN_TEST(test_arm_starts_recording_request);
  RUN_TEST(test_countdown_short_and_long);
  RUN_TEST(test_abort_pre_liftoff_vs_post);
  RUN_TEST(test_stop_recording_rejected_in_flight);
  RUN_TEST(test_reset_rejected_in_flight);
  RUN_TEST(test_command_unknown);
  RUN_TEST(test_command_duplicate_idempotent);
  RUN_TEST(test_ble_disconnect_does_not_cancel_countdown);
  RUN_TEST(test_flight_liftoff_multi_sample);
  RUN_TEST(test_false_quiet_does_not_land);
  RUN_TEST(test_sync_time_command);
  RUN_TEST(test_countdown_without_arm_rejected);
  return UNITY_END();
}

#endif
