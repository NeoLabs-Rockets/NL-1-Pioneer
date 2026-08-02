#include "event_log.h"
#include "time_manager.h"
#include "config.h"

#ifndef UNIT_TEST
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#endif

#include <stdio.h>
#include <string.h>

namespace EventLog {
namespace {

#ifndef UNIT_TEST
QueueHandle_t q = nullptr;
#endif
uint32_t drops = 0;

// Do not name this LINE_MAX — POSIX limits.h defines that as a macro.
static constexpr size_t kEventLineMax = 384;

const char* eventTypeName(EventType t) {
  switch (t) {
    case EventType::BOOT: return "BOOT";
    case EventType::STATE_CHANGE: return "STATE_CHANGE";
    case EventType::COMMAND: return "COMMAND";
    case EventType::COMMAND_RESULT: return "COMMAND_RESULT";
    case EventType::BLE_CONNECT: return "BLE_CONNECT";
    case EventType::BLE_DISCONNECT: return "BLE_DISCONNECT";
    case EventType::SENSOR_FAULT: return "SENSOR_FAULT";
    case EventType::CAMERA_FAULT: return "CAMERA_FAULT";
    case EventType::STORAGE_FAULT: return "STORAGE_FAULT";
    case EventType::FLIGHT_PHASE: return "FLIGHT_PHASE";
    case EventType::LIFTOFF: return "LIFTOFF";
    case EventType::APOGEE: return "APOGEE";
    case EventType::LANDING: return "LANDING";
    case EventType::ABORT: return "ABORT";
    case EventType::RECORDING_START: return "RECORDING_START";
    case EventType::RECORDING_STOP: return "RECORDING_STOP";
    case EventType::TIME_SYNC: return "TIME_SYNC";
    case EventType::WARNING: return "WARNING";
    case EventType::ERROR: return "ERROR";
    case EventType::RESET_REASON: return "RESET_REASON";
    case EventType::HEARTBEAT: return "HEARTBEAT";
    case EventType::GROUND_TEST: return "GROUND_TEST";
    default: return "UNKNOWN";
  }
}

void formatLine(char* out, size_t out_len, EventType type, const char* message, const char* extra) {
  TimeStamp ts = TimeManager::now();
  snprintf(out, out_len,
           "{\"mono_us\":%llu,\"session_us\":%llu,\"unix_ms\":%lld,\"tq\":%u,"
           "\"type\":\"%s\",\"msg\":\"%s\"%s%s}\n",
           static_cast<unsigned long long>(ts.mono_us),
           static_cast<unsigned long long>(ts.session_us),
           static_cast<long long>(ts.sync_unix_ms),
           static_cast<unsigned>(ts.time_quality),
           eventTypeName(type),
           message ? message : "",
           extra && extra[0] ? "," : "",
           extra && extra[0] ? extra : "");
}

}  // namespace

void begin() {
  drops = 0;
#ifndef UNIT_TEST
  if (!q) q = xQueueCreate(EVENT_QUEUE_LEN, kEventLineMax);
#endif
}

void emit(EventType type, const char* message, const char* extra_json) {
  char line[kEventLineMax];
  formatLine(line, sizeof(line), type, message, extra_json);
#ifndef UNIT_TEST
  if (!q) {
    drops++;
    return;
  }
  if (xQueueSend(q, line, 0) != pdTRUE) drops++;
#else
  (void)line;
#endif
}

void emitStateChange(MissionState from, MissionState to, const char* cause) {
  char extra[160];
  snprintf(extra, sizeof(extra),
           "\"from\":\"%s\",\"to\":\"%s\",\"cause\":\"%s\"",
           missionStateName(from), missionStateName(to), cause ? cause : "");
  emit(EventType::STATE_CHANGE, "state_change", extra);
}

void emitCommand(const char* cmd, uint32_t ext_msg_id, const char* payload,
                 CmdResult result, const char* detail) {
  const char* r =
      result == CmdResult::ACK ? "ACK" :
      result == CmdResult::NACK ? "NACK" :
      result == CmdResult::IGNORED ? "IGNORED" : "REJECTED";
  char extra[280];
  snprintf(extra, sizeof(extra),
           "\"cmd\":\"%s\",\"ext_id\":%lu,\"result\":\"%s\",\"detail\":\"%s\",\"payload\":%s",
           cmd ? cmd : "",
           static_cast<unsigned long>(ext_msg_id),
           r,
           detail ? detail : "",
           payload && payload[0] ? payload : "null");
  emit(EventType::COMMAND, cmd ? cmd : "cmd", extra);
}

uint32_t droppedCount() { return drops; }

bool pop(char* out, size_t out_len) {
#ifndef UNIT_TEST
  if (!q || !out || out_len < 2) return false;
  char buf[kEventLineMax];
  if (xQueueReceive(q, buf, 0) != pdTRUE) return false;
  strncpy(out, buf, out_len - 1);
  out[out_len - 1] = '\0';
  return true;
#else
  (void)out; (void)out_len;
  return false;
#endif
}

}  // namespace EventLog
