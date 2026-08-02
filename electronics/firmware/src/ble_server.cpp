#include "ble_server.h"
#include "command_handler.h"
#include "config.h"
#include "mission_fsm.h"
#include "event_log.h"
#include "time_manager.h"
#include "storage.h"
#include "camera_recorder.h"
#include "sensors.h"

#ifndef UNIT_TEST
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#endif

#include <stdio.h>
#include <string.h>

namespace BleServer {
namespace {

#ifndef UNIT_TEST
NimBLECharacteristic* statusChar = nullptr;
NimBLECharacteristic* telemetryChar = nullptr;
NimBLECharacteristic* eventChar = nullptr;
NimBLECharacteristic* commandChar = nullptr;

QueueHandle_t cmd_q = nullptr;
static constexpr size_t CMD_MAX = 256;

int connected_count = 0;
uint32_t last_rx_ms = 0;
uint32_t telem_hz = BLE_TELEMETRY_HZ;
uint32_t last_telem_ms = 0;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    connected_count++;
    MissionFsm::setBluetoothConnected(true);
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    if (connected_count > 0) connected_count--;
    if (connected_count == 0) MissionFsm::setBluetoothConnected(false);
    NimBLEDevice::getAdvertising()->start();
  }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    std::string v = c->getValue();
    if (v.empty() || !cmd_q) return;
    last_rx_ms = static_cast<uint32_t>(TimeManager::monoMs());
    char buf[CMD_MAX];
    size_t n = v.size() < CMD_MAX - 1 ? v.size() : CMD_MAX - 1;
    memcpy(buf, v.data(), n);
    buf[n] = '\0';
    xQueueSend(cmd_q, buf, 0);
  }
};
#endif

}  // namespace

void begin() {
#ifndef UNIT_TEST
  cmd_q = xQueueCreate(BLE_CMD_QUEUE_LEN, CMD_MAX);
  telem_hz = BLE_TELEMETRY_HZ;

  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setMTU(BLE_MTU_TARGET);
  NimBLEDevice::setPower(9);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService* service = server->createService(BLE_SERVICE_UUID);
  commandChar = service->createCharacteristic(
      BLE_COMMAND_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  commandChar->setCallbacks(new CommandCallbacks());

  statusChar = service->createCharacteristic(
      BLE_STATUS_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  telemetryChar = service->createCharacteristic(
      BLE_TELEMETRY_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  eventChar = service->createCharacteristic(
      BLE_EVENT_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  service->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->enableScanResponse(true);
  adv->setName(BLE_DEVICE_NAME);
  adv->start();
#endif
}

void end() {
#ifndef UNIT_TEST
  NimBLEDevice::deinit(true);
#endif
}

BluetoothState state() {
#ifndef UNIT_TEST
  if (connected_count > 0) return BluetoothState::CONNECTED;
  return BluetoothState::ADVERTISING;
#else
  return BluetoothState::OFF;
#endif
}

bool connected() {
#ifndef UNIT_TEST
  return connected_count > 0;
#else
  return false;
#endif
}

uint32_t lastRxMs() {
#ifndef UNIT_TEST
  return last_rx_ms;
#else
  return 0;
#endif
}

void setTelemetryHz(uint32_t hz) {
#ifndef UNIT_TEST
  if (hz < BLE_TELEMETRY_HZ_MIN) hz = BLE_TELEMETRY_HZ_MIN;
  if (hz > BLE_TELEMETRY_HZ) hz = BLE_TELEMETRY_HZ;
  telem_hz = hz;
#else
  (void)hz;
#endif
}

uint32_t telemetryHz() {
#ifndef UNIT_TEST
  return telem_hz;
#else
  return BLE_TELEMETRY_HZ;
#endif
}

void publishStatus(const SystemSnapshot& snap, const SensorSnapshot& sensors) {
#ifndef UNIT_TEST
  if (!statusChar) return;
  char data[360];
  const int32_t left = snap.countdown_left_ms;
  snprintf(data, sizeof(data),
    "{\"v\":%d,\"m\":\"%s\",\"r\":%u,\"ph\":\"%s\",\"ble\":%u,\"sd\":%u,\"cam\":%u,"
    "\"rec\":%d,\"left\":%ld,\"ign\":%lld,\"lo\":%lld,\"fps\":%.1f,\"fw\":%lu,\"fd\":%lu,"
    "\"free_kb\":%lu,\"imu\":%u,\"baro\":%u,\"e\":\"%s\",\"w\":\"%s\",\"u\":%lu,"
    "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"amag\":%.2f,"
    "\"p\":%.1f,\"alt\":%.2f,\"vz\":%.2f,"
    "\"tb\":%.1f,\"ti\":%.1f,\"tm\":%.1f,\"gt\":%d}",
    PROTOCOL_VERSION_MAJOR,
    missionStateName(snap.mission),
    static_cast<unsigned>(snap.recording),
    flightPhaseName(snap.phase),
    static_cast<unsigned>(snap.bluetooth),
    static_cast<unsigned>(snap.storage),
    static_cast<unsigned>(snap.camera),
    snap.recording == RecordingState::ACTIVE || snap.recording == RecordingState::POST_LANDING ? 1 : 0,
    static_cast<long>(left),
    static_cast<long long>(snap.expected_ignition_mono_ms),
    static_cast<long long>(snap.actual_liftoff_mono_ms),
    snap.actual_fps,
    static_cast<unsigned long>(snap.frames_written),
    static_cast<unsigned long>(snap.frames_dropped),
    static_cast<unsigned long>(snap.free_sd_bytes / 1024ULL),
    static_cast<unsigned>(sensors.imu_health),
    static_cast<unsigned>(sensors.baro_health),
    snap.last_error,
    snap.last_warning,
    static_cast<unsigned long>(snap.uptime_ms),
    sensors.ax, sensors.ay, sensors.az, sensors.accel_mag,
    sensors.pressure_pa, sensors.altitude_m, sensors.vert_vel_ms,
    sensors.temps[0].valid ? sensors.temps[0].celsius : -999.f,
    sensors.temps[1].valid ? sensors.temps[1].celsius : -999.f,
    sensors.temps[2].valid ? sensors.temps[2].celsius : -999.f,
    snap.ground_test ? 1 : 0
  );
  statusChar->setValue(reinterpret_cast<uint8_t*>(data), strlen(data));
  if (connected_count > 0) statusChar->notify();
#else
  (void)snap; (void)sensors;
#endif
}

void publishTelemetry(const SensorSnapshot& sensors, const SystemSnapshot& snap) {
#ifndef UNIT_TEST
  if (!telemetryChar || connected_count == 0) return;
  uint32_t now = static_cast<uint32_t>(TimeManager::monoMs());
  uint32_t period = telem_hz ? (1000u / telem_hz) : 1000u;
  if (now - last_telem_ms < period) return;
  last_telem_ms = now;

  // Compact live telemetry (keep under MTU)
  char data[220];
  snprintf(data, sizeof(data),
    "{\"t\":%llu,\"m\":%u,\"ph\":%u,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"g\":%.2f,"
    "\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,\"p\":%.0f,\"h\":%.1f,\"vz\":%.1f,\"fps\":%.1f}",
    static_cast<unsigned long long>(sensors.ts.mono_us / 1000ULL),
    static_cast<unsigned>(snap.mission),
    static_cast<unsigned>(snap.phase),
    sensors.ax, sensors.ay, sensors.az, sensors.accel_mag,
    sensors.gx, sensors.gy, sensors.gz,
    sensors.pressure_pa, sensors.altitude_m, sensors.vert_vel_ms,
    snap.actual_fps);
  telemetryChar->setValue(reinterpret_cast<uint8_t*>(data), strlen(data));
  telemetryChar->notify();
#else
  (void)sensors; (void)snap;
#endif
}

void serviceCommands() {
#ifndef UNIT_TEST
  if (!cmd_q) return;
  char body[CMD_MAX];
  while (xQueueReceive(cmd_q, body, 0) == pdTRUE) {
    char response[320];
    CommandHandler::handleJson(body, response, sizeof(response));
    if (statusChar) {
      // Piggy-back command result on status characteristic for simplicity;
      // also mirror to event characteristic.
      if (eventChar && connected_count > 0) {
        eventChar->setValue(reinterpret_cast<uint8_t*>(response), strlen(response));
        eventChar->notify();
      }
    }
    // Always refresh full status after a command
    // (caller may also publish; lightweight re-notify of result as status update)
  }
#endif
}

}  // namespace BleServer
