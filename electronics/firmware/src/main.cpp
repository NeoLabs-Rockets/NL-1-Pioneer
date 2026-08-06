/**
 * NeoLabs NL-1 Pioneer — Onboard Rocket Computer
 *
 * Records video + telemetry to microSD. Communicates with Mission Dashboard
 * over BLE. Never controls ignition or pyrotechnics.
 *
 * Hardware (connections.md + schematic):
 *   Seeed XIAO ESP32-S3 Sense
 *   BMP580 + LSM6DSO32 on I2C GPIO5/GPIO6
 *   Onboard camera + microSD
 */

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_system.h>

#include "config.h"
#include "board_pins.h"
#include "types.h"
#include "time_manager.h"
#include "event_log.h"
#include "mission_fsm.h"
#include "mission_fsm_internal.h"
#include "sensors.h"
#include "flight_detect.h"
#include "storage.h"
#include "camera_recorder.h"
#include "ble_server.h"
#include "ota_service.h"
#include "crc16.h"

static SensorSnapshot g_sensors{};
static SystemSnapshot g_system{};
static portMUX_TYPE g_snap_mux = portMUX_INITIALIZER_UNLOCKED;

// ── Status LED (XIAO amber user LED, GPIO21, active-low) ────────────────────
// On Sense, GPIO21 is also microSD CS. Never drive it as an LED — any
// pre-mount bit-banging of CS contributes to "Card Failed! cmd: 0x00".
#if USER_LED_AVAILABLE
static void statusLedInit() {}
static void statusLedBootFlash() {}
static void statusLedReleaseForSd() {
  // Leave CS idle high before Storage::begin claims it.
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
}
static void statusLedHeartbeat() {}
#else
static void statusLedInit() {}
static void statusLedBootFlash() {}
static void statusLedReleaseForSd() {}
static void statusLedHeartbeat() {}
#endif

static void fillSystemSnapshot() {
  CameraRecorder::Stats cs = CameraRecorder::stats();
  g_system.mission = MissionFsm::state();
  g_system.recording = MissionFsm::recordingState();
  g_system.storage = Storage::state();
  g_system.bluetooth = BleServer::state();
  g_system.camera = CameraRecorder::state();
  g_system.phase = FlightDetect::phase();
  g_system.uptime_ms = static_cast<uint32_t>(TimeManager::monoMs());
  g_system.countdown_left_ms = MissionFsm::countdownRemainingMs(TimeManager::monoMs());
  g_system.expected_ignition_mono_ms = MissionFsm::expectedIgnitionMonoMs();
  g_system.actual_liftoff_mono_ms = MissionFsm::actualLiftoffMonoMs();
  g_system.frames_written = cs.frames_written;
  g_system.frames_dropped = cs.frames_dropped;
  g_system.actual_fps = cs.actual_fps;
  g_system.free_sd_bytes = Storage::freeBytes();
  g_system.session_bytes = 0;
  g_system.camera_profile = static_cast<uint8_t>(cs.profile);
  g_system.ground_test = false;
  g_system.last_error[0] = '\0';
  g_system.last_warning[0] = '\0';
}

static void buildTelemetryRecord(TelemetryRecord& rec, uint32_t seq) {
  TimeStamp ts = TimeManager::now();
  memset(&rec, 0, sizeof(rec));
  rec.magic = TELEM_MAGIC;
  rec.version = 1;
  rec.size = sizeof(TelemetryRecord);
  rec.mono_us = ts.mono_us;
  rec.session_us = ts.session_us;
  rec.sync_unix_ms = ts.sync_unix_ms;
  portENTER_CRITICAL(&g_snap_mux);
  rec.ax = g_sensors.ax;
  rec.ay = g_sensors.ay;
  rec.az = g_sensors.az;
  rec.gx = g_sensors.gx;
  rec.gy = g_sensors.gy;
  rec.gz = g_sensors.gz;
  rec.accel_mag = g_sensors.accel_mag;
  rec.pressure_pa = g_sensors.pressure_pa;
  rec.altitude_m = g_sensors.altitude_m;
  rec.vert_vel_ms = g_sensors.vert_vel_ms;
  rec.temp_bmp_c = g_sensors.temps[0].valid ? g_sensors.temps[0].celsius : NAN;
  rec.temp_imu_c = g_sensors.temps[1].valid ? g_sensors.temps[1].celsius : NAN;
  rec.temp_mcu_c = g_sensors.temps[2].valid ? g_sensors.temps[2].celsius : NAN;
  uint8_t flags = 0;
  if (g_sensors.imu_health == SensorHealth::OK) flags |= 1;
  if (g_sensors.baro_health == SensorHealth::OK) flags |= 2;
  if (CameraRecorder::state() == CameraState::RECORDING
      || CameraRecorder::state() == CameraState::READY
      || CameraRecorder::state() == CameraState::FALLBACK) flags |= 4;
  if (Storage::state() == StorageState::MOUNTED || Storage::state() == StorageState::WRITING) flags |= 8;
  portEXIT_CRITICAL(&g_snap_mux);
  rec.mission_state = static_cast<uint8_t>(MissionFsm::state());
  rec.recording_state = static_cast<uint8_t>(MissionFsm::recordingState());
  rec.flight_phase = static_cast<uint8_t>(FlightDetect::phase());
  rec.flags = flags;
  CameraRecorder::Stats cs = CameraRecorder::stats();
  rec.frame_count = cs.frames_written;
  rec.actual_fps_x10 = static_cast<uint16_t>(cs.actual_fps * 10.f);
  rec.dropped_frames = static_cast<uint16_t>(min(cs.frames_dropped, 65535u));
  rec.free_sd_kb = static_cast<uint32_t>(Storage::freeBytes() / 1024ULL);
  rec.seq = seq;
  rec.crc16 = crc16_ccitt(reinterpret_cast<const uint8_t*>(&rec),
                          sizeof(TelemetryRecord) - sizeof(uint16_t));
}

static void logResetReason() {
  esp_reset_reason_t r = esp_reset_reason();
  const char* name = "UNKNOWN";
  switch (r) {
    case ESP_RST_POWERON: name = "POWERON"; break;
    case ESP_RST_SW: name = "SW"; break;
    case ESP_RST_PANIC: name = "PANIC"; break;
    case ESP_RST_INT_WDT: name = "INT_WDT"; break;
    case ESP_RST_TASK_WDT: name = "TASK_WDT"; break;
    case ESP_RST_WDT: name = "WDT"; break;
    case ESP_RST_BROWNOUT: name = "BROWNOUT"; break;
    case ESP_RST_SDIO: name = "SDIO"; break;
    default: break;
  }
  char msg[48];
  snprintf(msg, sizeof(msg), "reset_%s", name);
  EventLog::emit(EventType::RESET_REASON, msg);
  EventLog::emit(EventType::BOOT, FIRMWARE_VERSION);
}

// ── Tasks ───────────────────────────────────────────────────────────────────

static void sensorTask(void*) {
  // Subscribe before the loop: esp_task_wdt_reset() is a no-op (ESP_ERR_NOT_FOUND)
  // for any task that was never added, so an unsubscribed task is unmonitored.
  esp_task_wdt_add(nullptr);
  const TickType_t period = pdMS_TO_TICKS(1000 / SENSOR_HZ_IMU);
  TickType_t last = xTaskGetTickCount();
  uint32_t telem_div = 0;
  uint32_t seq = 0;
  for (;;) {
    SensorSnapshot s;
    if (Sensors::sample(s)) {
      portENTER_CRITICAL(&g_snap_mux);
      g_sensors = s;
      portEXIT_CRITICAL(&g_snap_mux);

      bool care = MissionFsm::countdownActive()
                  || MissionFsm::state() == MissionState::ARMED
                  || MissionFsm::state() == MissionState::RECORDING
                  || MissionFsm::state() == MissionState::COUNTDOWN
                  || MissionFsm::state() == MissionState::IGNITION_EXPECTED
                  || MissionFsm::hasLiftedOff();
      if (FlightDetect::update(s, MissionFsm::expectedIgnitionMonoMs(), care)) {
        MissionFsm::setFlightPhase(FlightDetect::phase(), "sensor");
      }

      // Log telemetry at TELEMETRY_LOG_HZ when session open
      telem_div++;
      if (telem_div >= (SENSOR_HZ_IMU / TELEMETRY_LOG_HZ)) {
        telem_div = 0;
        if (Storage::isSessionOpen()) {
          TelemetryRecord rec;
          buildTelemetryRecord(rec, seq++);
          Storage::enqueueTelemetry(rec);
        }
      }
    }
    esp_task_wdt_reset();
    vTaskDelayUntil(&last, period);
  }
}

static void storageTask(void*) {
  esp_task_wdt_add(nullptr);
  for (;;) {
    // Session start/stop driven by mission flags
    char reason[40];
    if (MissionFsm::consumeStartRecording(reason, sizeof(reason))) {
      if (!Storage::isSessionOpen()) {
        if (Storage::openSession(reason)) {
          CameraRecorder::start();
          MissionFsm::markRecordingActive();
        } else {
          MissionFsm::markRecordingFailed();
          EventLog::emit(EventType::STORAGE_FAULT, "session_open_failed");
        }
      } else {
        // Idempotent — already recording
        MissionFsm::markRecordingActive();
      }
    }
    if (MissionFsm::consumeStopRecording()) {
      CameraRecorder::stop();
      Storage::closeSession("stop");
      MissionFsm::markRecordingStopped();
    }
    Storage::taskLoop();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void cameraTask(void*) {
  esp_task_wdt_add(nullptr);
  for (;;) {
    if (CameraRecorder::isRecording()) {
      CameraRecorder::service();
    }
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void missionTask(void*) {
  esp_task_wdt_add(nullptr);
  uint32_t last_status_ms = 0;
  for (;;) {
    BleServer::serviceCommands();
    OtaService::loop();
    MissionFsm::tick(TimeManager::monoMs());

    fillSystemSnapshot();

    uint32_t now = static_cast<uint32_t>(TimeManager::monoMs());
    SensorSnapshot sensors;
    portENTER_CRITICAL(&g_snap_mux);
    sensors = g_sensors;
    portEXIT_CRITICAL(&g_snap_mux);

    // Status every 1 s; telemetry at configured rate
    if (now - last_status_ms >= 1000) {
      last_status_ms = now;
      BleServer::publishStatus(g_system, sensors);
    }
    BleServer::publishTelemetry(sensors, g_system);

    // Shed BLE telemetry rate if SD is falling behind (simple heuristic)
    CameraRecorder::Stats cs = CameraRecorder::stats();
    if (cs.frames_dropped > 0 && cs.frames_written > 0
        && cs.frames_dropped * 10 > cs.frames_written) {
      BleServer::setTelemetryHz(BLE_TELEMETRY_HZ_MIN);
    }

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n[NL1] Rocket Computer %s\n", FIRMWARE_VERSION);

  // Visual proof-of-life before SD claims GPIO21 as CS.
  statusLedInit();
  statusLedBootFlash();

  TimeManager::begin();
  EventLog::begin();
  MissionFsm::begin();
  MissionFsm::advanceFromBoot();
  logResetReason();

  // Task watchdog (timeout seconds, panic on trip)
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  // Release GPIO21 to SD CS before mounting the card.
  statusLedReleaseForSd();
  bool sd_ok = Storage::begin();

  // Camera SCCB (pins 39/40) shares the ESP32-S3 I2C controller with Wire.
  // If sensors are brought up first, esp_camera_init() leaves external I2C
  // (GPIO5/6) broken — scan/begin OK, then continuous Wire Error -1.
  // Init camera first, then external sensors on Wire.
  bool cam_ok = CameraRecorder::begin();
  bool sens_ok = Sensors::begin();
  FlightDetect::begin();
  BleServer::begin();

  if (!sens_ok) EventLog::emit(EventType::WARNING, "no_sensors");
  if (!sd_ok) EventLog::emit(EventType::WARNING, "no_sd");
  if (!cam_ok) EventLog::emit(EventType::WARNING, "no_camera");

  // Self-test complete — enter IDLE even if some peripherals missing (degraded)
  MissionFsm::advanceSelfTestOk();

  xTaskCreatePinnedToCore(sensorTask, "sensor", 6144, nullptr, PRIO_SENSOR, nullptr, 1);
  xTaskCreatePinnedToCore(storageTask, "storage", 8192, nullptr, PRIO_STORAGE, nullptr, 1);
  xTaskCreatePinnedToCore(cameraTask, "camera", 8192, nullptr, PRIO_CAMERA, nullptr, 0);
  xTaskCreatePinnedToCore(missionTask, "mission", 8192, nullptr, PRIO_MISSION, nullptr, 0);

  Serial.println("[NL1] ready — advertising as NeoLabs Rocket Computer");
  Serial.printf("[NL1] SD=%d (free=%llu MB) IMU=%d BARO=%d CAM=%d\n",
                sd_ok,
                static_cast<unsigned long long>(Storage::freeBytes() / (1024ULL * 1024ULL)),
                Sensors::imuPresent(), Sensors::baroPresent(), cam_ok);
  Serial.printf("[NL1] storage_state=%d camera_state=%d\n",
                static_cast<int>(Storage::state()),
                static_cast<int>(CameraRecorder::state()));
}

void loop() {
  statusLedHeartbeat();
  OtaService::loop();
  esp_task_wdt_reset();
  vTaskDelay(pdMS_TO_TICKS(20));
}
