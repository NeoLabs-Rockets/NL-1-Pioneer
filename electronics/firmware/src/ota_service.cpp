#include "ota_service.h"
#include "config.h"
#include "mission_fsm.h"
#include "event_log.h"
#include "types.h"

#ifndef UNIT_TEST
#include <Arduino.h>
#include <Update.h>
#include <NimBLEDevice.h>
#include <mbedtls/sha256.h>
#include <esp_ota_ops.h>
#include <ctype.h>
#endif

#include <stdio.h>
#include <string.h>

namespace OtaService {
namespace {

#ifndef UNIT_TEST
NimBLECharacteristic* otaStatusChar = nullptr;

bool otaActive = false;
bool otaShaInitialized = false;
uint32_t otaExpectedSize = 0;
uint32_t otaReceived = 0;
uint32_t otaLastReported = 0;
unsigned long otaLastActivityAt = 0;
unsigned long otaRebootAt = 0;
uint16_t otaConnectionHandle = 0xFFFF;
char otaExpectedSha[65] = "";
char otaVersion[64] = "";
char otaState[16] = "idle";
char otaError[48] = "";
mbedtls_sha256_context otaShaContext;

static constexpr unsigned long OTA_IDLE_TIMEOUT_MS = 15000UL;
static constexpr unsigned long OTA_REBOOT_DELAY_MS = 1200UL;
static constexpr uint32_t OTA_PROGRESS_INTERVAL_BYTES = 2560UL;

String jsonStringField(const String& src, const char* key) {
  String pattern = String("\"") + key + "\":\"";
  int start = src.indexOf(pattern);
  if (start < 0) return "";
  start += pattern.length();
  int end = src.indexOf('"', start);
  return end < 0 ? "" : src.substring(start, end);
}

int jsonIntField(const String& src, const char* key, int fallback) {
  String pattern = String("\"") + key + "\":";
  int start = src.indexOf(pattern);
  if (start < 0) return fallback;
  start += pattern.length();
  while (start < (int)src.length() && src[start] == ' ') start++;
  int end = start;
  while (end < (int)src.length() && isDigit(src[end])) end++;
  return end == start ? fallback : src.substring(start, end).toInt();
}

bool validSha256(const String& value) {
  if (value.length() != 64) return false;
  for (size_t i = 0; i < value.length(); i++) {
    const char c = value[i];
    if (!isDigit(c) && !(c >= 'a' && c <= 'f') && !(c >= 'A' && c <= 'F')) return false;
  }
  return true;
}

bool validFirmwareVersion(const String& value) {
  if (value.length() == 0 || value.length() > 63 || !isalnum((unsigned char)value[0])) return false;
  for (size_t i = 1; i < value.length(); i++) {
    const char c = value[i];
    if (!isalnum((unsigned char)c) && c != '.' && c != '_' && c != '+' && c != '-') return false;
  }
  return true;
}

bool deviceIdleForOta() {
  if (MissionFsm::recordingActive()) return false;
  if (MissionFsm::hasLiftedOff()) return false;
  if (missionInFlight(MissionFsm::state())) return false;
  if (MissionFsm::state() == MissionState::COUNTDOWN
      || MissionFsm::state() == MissionState::IGNITION_EXPECTED) {
    return false;
  }
  return true;
}

void publishOtaStatusInternal() {
  if (!otaStatusChar) return;
  char data[200];
  snprintf(data, sizeof(data),
           "{\"state\":\"%s\",\"received\":%lu,\"total\":%lu,\"error\":\"%s\",\"version\":\"%s\"}",
           otaState, static_cast<unsigned long>(otaReceived),
           static_cast<unsigned long>(otaExpectedSize), otaError, otaVersion);
  otaStatusChar->setValue(reinterpret_cast<uint8_t*>(data), strlen(data));
  otaStatusChar->notify();
}

void resetOtaState(const char* state, const char* error = "") {
  if (Update.isRunning()) Update.abort();
  if (otaShaInitialized) {
    mbedtls_sha256_free(&otaShaContext);
    otaShaInitialized = false;
  }
  otaActive = false;
  otaExpectedSize = 0;
  otaReceived = 0;
  otaLastReported = 0;
  otaConnectionHandle = 0xFFFF;
  otaExpectedSha[0] = '\0';
  otaVersion[0] = '\0';
  strncpy(otaState, state ? state : "idle", sizeof(otaState) - 1);
  otaState[sizeof(otaState) - 1] = '\0';
  strncpy(otaError, error ? error : "", sizeof(otaError) - 1);
  otaError[sizeof(otaError) - 1] = '\0';
  publishOtaStatusInternal();
}

void failOta(const char* error) {
  resetOtaState("error", error);
  EventLog::emit(EventType::ERROR, "ota_failed", error);
}

class OtaControlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    const std::string raw = characteristic->getValue();
    onControlWrite(raw.c_str(), connInfo.getConnHandle());
  }
};

class OtaDataCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    const std::string value = characteristic->getValue();
    onDataWrite(reinterpret_cast<const uint8_t*>(value.data()), value.size(), connInfo.getConnHandle());
  }
};
#endif

}  // namespace

void begin() {}

void attachToService(void* nimbleService) {
#ifndef UNIT_TEST
  auto* service = static_cast<NimBLEService*>(nimbleService);
  if (!service) return;

  NimBLECharacteristic* otaControl = service->createCharacteristic(
      BLE_OTA_CONTROL_UUID, NIMBLE_PROPERTY::WRITE);
  otaControl->setCallbacks(new OtaControlCallbacks());

  NimBLECharacteristic* otaData = service->createCharacteristic(
      BLE_OTA_DATA_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  otaData->setCallbacks(new OtaDataCallbacks());

  otaStatusChar = service->createCharacteristic(
      BLE_OTA_STATUS_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  resetOtaState("idle");

#ifdef CONFIG_APP_ROLLBACK_ENABLE
  const esp_partition_t* running = esp_ota_get_running_partition();
  esp_ota_img_states_t imgState;
  if (esp_ota_get_state_partition(running, &imgState) == ESP_OK
      && imgState == ESP_OTA_IMG_PENDING_VERIFY) {
    esp_ota_mark_app_valid_cancel_rollback();
  }
#endif
#else
  (void)nimbleService;
#endif
}

void publishStatus() {
#ifndef UNIT_TEST
  publishOtaStatusInternal();
#endif
}

bool active() {
#ifndef UNIT_TEST
  return otaActive;
#else
  return false;
#endif
}

bool rebootPending() {
#ifndef UNIT_TEST
  return otaRebootAt != 0;
#else
  return false;
#endif
}

void loop() {
#ifndef UNIT_TEST
  const unsigned long now = millis();
  if (otaActive && now - otaLastActivityAt > OTA_IDLE_TIMEOUT_MS) failOta("timeout");
  if (otaRebootAt && (long)(now - otaRebootAt) >= 0) {
    otaRebootAt = 0;
    ESP.restart();
  }
#endif
}

void onControlWrite(const char* json, uint16_t connHandle) {
#ifndef UNIT_TEST
  String body = json ? json : "";
  const String cmd = jsonStringField(body, "cmd");

  if (cmd == "status") {
    publishOtaStatusInternal();
    return;
  }
  if (otaRebootAt) {
    publishOtaStatusInternal();
    return;
  }
  if (cmd == "abort") {
    if (otaActive && connHandle != otaConnectionHandle) {
      strncpy(otaError, "not_owner", sizeof(otaError) - 1);
      publishOtaStatusInternal();
      return;
    }
    resetOtaState("aborted", "cancelled");
    return;
  }
  if (cmd == "begin") {
    if (otaActive || otaRebootAt) {
      strncpy(otaState, "error", sizeof(otaState) - 1);
      strncpy(otaError, "already_active", sizeof(otaError) - 1);
      publishOtaStatusInternal();
      return;
    }
    if (!deviceIdleForOta()) {
      strncpy(otaState, "error", sizeof(otaState) - 1);
      strncpy(otaError, "device_not_idle", sizeof(otaError) - 1);
      publishOtaStatusInternal();
      return;
    }

    const int requestedSize = jsonIntField(body, "size", 0);
    const String requestedSha = jsonStringField(body, "sha256");
    const String requestedVersion = jsonStringField(body, "version");
    if (requestedSize <= 0 || !validSha256(requestedSha) || !validFirmwareVersion(requestedVersion)) {
      strncpy(otaState, "error", sizeof(otaState) - 1);
      strncpy(otaError, "invalid_manifest", sizeof(otaError) - 1);
      publishOtaStatusInternal();
      return;
    }

    if (!Update.begin((size_t)requestedSize, U_FLASH)) {
      strncpy(otaState, "error", sizeof(otaState) - 1);
      strncpy(otaError, Update.errorString(), sizeof(otaError) - 1);
      publishOtaStatusInternal();
      return;
    }
    mbedtls_sha256_init(&otaShaContext);
    if (mbedtls_sha256_starts_ret(&otaShaContext, 0) != 0) {
      Update.abort();
      mbedtls_sha256_free(&otaShaContext);
      strncpy(otaState, "error", sizeof(otaState) - 1);
      strncpy(otaError, "sha_init_failed", sizeof(otaError) - 1);
      publishOtaStatusInternal();
      return;
    }
    otaShaInitialized = true;
    otaActive = true;
    otaExpectedSize = (uint32_t)requestedSize;
    otaReceived = 0;
    otaLastReported = 0;
    otaLastActivityAt = millis();
    otaConnectionHandle = connHandle;
    strncpy(otaExpectedSha, requestedSha.c_str(), sizeof(otaExpectedSha) - 1);
    for (char* p = otaExpectedSha; *p; p++) *p = tolower(*p);
    strncpy(otaVersion, requestedVersion.c_str(), sizeof(otaVersion) - 1);
    strncpy(otaState, "ready", sizeof(otaState) - 1);
    otaError[0] = '\0';
    EventLog::emit(EventType::WARNING, "ota_begin", otaVersion);
    publishOtaStatusInternal();
    return;
  }
  if (cmd == "finish") {
    if (!otaActive || connHandle != otaConnectionHandle) {
      strncpy(otaState, "error", sizeof(otaState) - 1);
      strncpy(otaError, otaActive ? "not_owner" : "not_active", sizeof(otaError) - 1);
      publishOtaStatusInternal();
      return;
    }
    if (otaReceived != otaExpectedSize) {
      failOta("incomplete");
      return;
    }
    uint8_t digest[32];
    if (!otaShaInitialized || mbedtls_sha256_finish_ret(&otaShaContext, digest) != 0) {
      failOta("sha_finish_failed");
      return;
    }
    mbedtls_sha256_free(&otaShaContext);
    otaShaInitialized = false;
    char actualSha[65];
    for (uint8_t i = 0; i < sizeof(digest); i++) snprintf(actualSha + i * 2, 3, "%02x", digest[i]);
    actualSha[64] = '\0';
    if (strcmp(otaExpectedSha, actualSha) != 0) {
      failOta("sha_mismatch");
      return;
    }
    if (!Update.end(false)) {
      failOta(Update.errorString());
      return;
    }
    otaActive = false;
    strncpy(otaState, "complete", sizeof(otaState) - 1);
    otaError[0] = '\0';
    publishOtaStatusInternal();
    EventLog::emit(EventType::WARNING, "ota_complete", otaVersion);
    otaRebootAt = millis() + OTA_REBOOT_DELAY_MS;
    return;
  }
  strncpy(otaState, "error", sizeof(otaState) - 1);
  strncpy(otaError, "unknown_command", sizeof(otaError) - 1);
  publishOtaStatusInternal();
#else
  (void)json; (void)connHandle;
#endif
}

void onDataWrite(const uint8_t* data, size_t len, uint16_t connHandle) {
#ifndef UNIT_TEST
  if (!otaActive || connHandle != otaConnectionHandle || !data || len <= 4) return;
  const uint32_t offset = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
      | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
  const size_t payloadSize = len - 4;
  if (offset < otaReceived) {
    publishOtaStatusInternal();
    return;
  }
  if (offset != otaReceived || otaReceived + payloadSize > otaExpectedSize) {
    failOta("invalid_offset");
    return;
  }
  if (Update.write(const_cast<uint8_t*>(data + 4), payloadSize) != payloadSize) {
    failOta(Update.errorString());
    return;
  }
  if (mbedtls_sha256_update_ret(&otaShaContext, data + 4, payloadSize) != 0) {
    failOta("sha_update_failed");
    return;
  }
  otaReceived += payloadSize;
  otaLastActivityAt = millis();
  strncpy(otaState, "receiving", sizeof(otaState) - 1);
  if (otaReceived == otaExpectedSize || otaReceived - otaLastReported >= OTA_PROGRESS_INTERVAL_BYTES) {
    otaLastReported = otaReceived;
    publishOtaStatusInternal();
  }
#else
  (void)data; (void)len; (void)connHandle;
#endif
}

}  // namespace OtaService
