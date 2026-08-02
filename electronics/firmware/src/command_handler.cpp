#include "command_handler.h"
#include "mission_fsm.h"
#include "mission_fsm_internal.h"
#include "time_manager.h"
#include "event_log.h"
#include "config.h"
#include "sensors.h"
#include "flight_detect.h"
#include "storage.h"
#include "ble_server.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

namespace CommandHandler {
namespace {

uint32_t id_cache[BLE_MSG_ID_CACHE];
size_t id_cache_i = 0;

// Minimal JSON helpers (same style as Launch Controller firmware)
const char* findKey(const char* src, const char* key) {
  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  return strstr(src, pattern);
}

bool jsonString(const char* src, const char* key, char* out, size_t out_len) {
  if (!src || !out || !out_len) return false;
  out[0] = '\0';
  const char* p = findKey(src, key);
  if (!p) return false;
  p = strchr(p + 1, ':');
  if (!p) return false;
  p++;
  while (*p == ' ') p++;
  if (*p != '"') return false;
  p++;
  size_t i = 0;
  while (*p && *p != '"' && i + 1 < out_len) out[i++] = *p++;
  out[i] = '\0';
  return true;
}

int jsonInt(const char* src, const char* key, int fallback) {
  const char* p = findKey(src, key);
  if (!p) return fallback;
  p = strchr(p + 1, ':');
  if (!p) return fallback;
  p++;
  while (*p == ' ') p++;
  return atoi(p);
}

int64_t jsonInt64(const char* src, const char* key, int64_t fallback) {
  const char* p = findKey(src, key);
  if (!p) return fallback;
  p = strchr(p + 1, ':');
  if (!p) return fallback;
  p++;
  while (*p == ' ') p++;
  return atoll(p);
}

double jsonDouble(const char* src, const char* key, double fallback) {
  const char* p = findKey(src, key);
  if (!p) return fallback;
  p = strchr(p + 1, ':');
  if (!p) return fallback;
  p++;
  while (*p == ' ') p++;
  return atof(p);
}

void writeResponse(char* out, size_t n, const char* cmd, CmdResult r, const char* detail,
                   uint32_t seq) {
  if (!out || !n) return;
  const char* rs =
      r == CmdResult::ACK ? "ACK" :
      r == CmdResult::NACK ? "NACK" :
      r == CmdResult::IGNORED ? "IGNORED" : "REJECTED";
  snprintf(out, n,
           "{\"v\":%d,\"cmd\":\"%s\",\"result\":\"%s\",\"detail\":\"%s\",\"seq\":%lu,\"mono_ms\":%llu,"
           "\"mission\":\"%s\",\"recording\":%d}",
           PROTOCOL_VERSION_MAJOR, cmd ? cmd : "", rs, detail ? detail : "",
           static_cast<unsigned long>(seq),
           static_cast<unsigned long long>(TimeManager::monoMs()),
           missionStateName(MissionFsm::state()),
           MissionFsm::recordingActive() ? 1 : 0);
}

}  // namespace

bool alreadyProcessed(uint32_t msg_id) {
  if (msg_id == 0) return false;
  for (size_t i = 0; i < BLE_MSG_ID_CACHE; i++) {
    if (id_cache[i] == msg_id) return true;
  }
  return false;
}

void rememberProcessed(uint32_t msg_id) {
  if (msg_id == 0) return;
  id_cache[id_cache_i % BLE_MSG_ID_CACHE] = msg_id;
  id_cache_i++;
}

CmdResult handleJson(const char* body, char* response_out, size_t response_len) {
  if (!body || !body[0]) {
    writeResponse(response_out, response_len, "?", CmdResult::NACK, "empty", 0);
    return CmdResult::NACK;
  }

  char cmd[40] = "";
  char sid[40] = "";
  jsonString(body, "cmd", cmd, sizeof(cmd));
  jsonString(body, "sid", sid, sizeof(sid));
  uint32_t seq = static_cast<uint32_t>(jsonInt(body, "seq", 0));
  uint32_t msg_id = static_cast<uint32_t>(jsonInt(body, "msg_id", static_cast<int>(seq)));

  if (!cmd[0]) {
    writeResponse(response_out, response_len, "?", CmdResult::NACK, "missing_cmd", seq);
    EventLog::emitCommand("?", msg_id, body, CmdResult::NACK, "missing_cmd");
    return CmdResult::NACK;
  }

  // Idempotency for critical commands
  const bool critical =
      !strcmp(cmd, Cmd::ARM) || !strcmp(cmd, Cmd::DISARM) ||
      !strcmp(cmd, Cmd::START_RECORDING) || !strcmp(cmd, Cmd::STOP_RECORDING) ||
      !strcmp(cmd, Cmd::START_COUNTDOWN) || !strcmp(cmd, Cmd::SET_EXPECTED_IGNITION) ||
      !strcmp(cmd, Cmd::CANCEL_COUNTDOWN) || !strcmp(cmd, Cmd::ABORT) ||
      !strcmp(cmd, Cmd::RESET);
  if (critical && alreadyProcessed(msg_id)) {
    writeResponse(response_out, response_len, cmd, CmdResult::IGNORED, "duplicate", seq);
    EventLog::emitCommand(cmd, msg_id, body, CmdResult::IGNORED, "duplicate");
    return CmdResult::IGNORED;
  }

  CmdResult result = CmdResult::NACK;
  const char* detail = "";

  if (!strcmp(cmd, Cmd::GET_STATUS) || !strcmp(cmd, "status")) {
    result = CmdResult::ACK;
    detail = "status";
  } else if (!strcmp(cmd, Cmd::GET_CAPABILITIES) || !strcmp(cmd, "capabilities")) {
    result = CmdResult::ACK;
    detail = "capabilities";
    if (response_out && response_len) {
      snprintf(response_out, response_len,
               "{\"v\":%d,\"cmd\":\"capabilities\",\"result\":\"ACK\","
               "\"board\":\"%s\",\"fw\":\"%s\",\"imu\":\"LSM6DSO32\",\"baro\":\"BMP580\","
               "\"camera\":true,\"sd\":true,\"battery_adc\":false,\"file_transfer\":true,"
               "\"chunk\":%u,\"protocol\":\"%d.%d\",\"ignition_control\":false}",
               PROTOCOL_VERSION_MAJOR, BOARD_MODEL_NAME, FIRMWARE_VERSION,
               static_cast<unsigned>(BLE_FILE_CHUNK_MAX),
               PROTOCOL_VERSION_MAJOR, PROTOCOL_VERSION_MINOR);
      EventLog::emitCommand(cmd, msg_id, body, result, detail);
      if (critical) rememberProcessed(msg_id);
      return result;
    }
  } else if (!strcmp(cmd, Cmd::SYNC_TIME) || !strcmp(cmd, "sync_time")) {
    int64_t unix_ms = jsonInt64(body, "unix_ms", 0);
    uint32_t rtt = static_cast<uint32_t>(jsonInt(body, "rtt_ms", 0));
    if (unix_ms <= 0) {
      result = CmdResult::NACK;
      detail = "bad_time";
    } else {
      TimeManager::applySync(unix_ms, rtt, TimeManager::monoMs());
      EventLog::emit(EventType::TIME_SYNC, "synced");
      result = CmdResult::ACK;
      detail = "synced";
    }
  } else if (!strcmp(cmd, Cmd::PING) || !strcmp(cmd, "ping")) {
    result = CmdResult::ACK;
    detail = "pong";
  } else if (!strcmp(cmd, Cmd::PREPARE_RECORDING)) {
    result = MissionFsm::cmdPrepareRecording();
  } else if (!strcmp(cmd, Cmd::ARM) || !strcmp(cmd, "arm")) {
    result = MissionFsm::cmdArm(sid);
  } else if (!strcmp(cmd, Cmd::DISARM) || !strcmp(cmd, "disarm")) {
    result = MissionFsm::cmdDisarm(sid);
  } else if (!strcmp(cmd, Cmd::START_RECORDING)) {
    result = MissionFsm::cmdStartRecording("ble");
  } else if (!strcmp(cmd, Cmd::STOP_RECORDING)) {
    result = MissionFsm::cmdStopRecording("ble", false);
  } else if (!strcmp(cmd, Cmd::START_COUNTDOWN) || !strcmp(cmd, "countdown_start")) {
    int seconds = jsonInt(body, "seconds", 10);
    if (seconds < 1) seconds = 1;
    if (seconds > 600) seconds = 600;
    uint32_t duration_ms = static_cast<uint32_t>(seconds) * 1000u;
    int64_t ign = jsonInt64(body, "ignition_mono_ms", -1);
    if (ign < 0) {
      // Prefer absolute unix → mono if provided
      int64_t ign_unix = jsonInt64(body, "ignition_unix_ms", -1);
      if (ign_unix > 0 && TimeManager::isSynced()) {
        // mono_ms = unix_ms - offset ≈ reverse of applySync
        ign = ign_unix - (TimeManager::syncUnixMs() - static_cast<int64_t>(TimeManager::monoMs()));
      } else {
        ign = static_cast<int64_t>(TimeManager::monoMs()) + duration_ms;
      }
    }
    result = MissionFsm::cmdStartCountdown(duration_ms, ign);
  } else if (!strcmp(cmd, Cmd::UPDATE_COUNTDOWN) || !strcmp(cmd, "countdown_update")) {
    int left = jsonInt(body, "left", -1);
    int64_t ign = jsonInt64(body, "ignition_mono_ms", -1);
    result = MissionFsm::cmdUpdateCountdown(left >= 0 ? static_cast<uint32_t>(left) * 1000u : 0, ign);
  } else if (!strcmp(cmd, Cmd::SET_EXPECTED_IGNITION)) {
    int64_t ign = jsonInt64(body, "ignition_mono_ms", -1);
    result = MissionFsm::cmdSetExpectedIgnition(ign);
  } else if (!strcmp(cmd, Cmd::CANCEL_COUNTDOWN) || !strcmp(cmd, "countdown_cancel")) {
    result = MissionFsm::cmdCancelCountdown("ble");
  } else if (!strcmp(cmd, Cmd::ABORT) || !strcmp(cmd, "abort")) {
    char reason[48] = "";
    jsonString(body, "reason", reason, sizeof(reason));
    result = MissionFsm::cmdAbort(reason[0] ? reason : "ble_abort");
  } else if (!strcmp(cmd, Cmd::RESET) || !strcmp(cmd, "reset")) {
    int force = jsonInt(body, "force", 0);
    result = MissionFsm::cmdReset(force == 1);
  } else if (!strcmp(cmd, Cmd::GROUND_TEST) || !strcmp(cmd, "ground_test")) {
    // Ground-test simulation commands: {"cmd":"ground_test","scenario":"full_flight"}
    char scenario[40] = "";
    jsonString(body, "scenario", scenario, sizeof(scenario));
    Sensors::setSimulation(true);
    EventLog::emit(EventType::GROUND_TEST, scenario[0] ? scenario : "enable");
    // Inject a simple liftoff impulse sample for immediate testing
    if (!strcmp(scenario, "liftoff") || !strcmp(scenario, "full_flight")) {
      SensorSnapshot s{};
      s.valid = true;
      s.ax = 0; s.ay = 0; s.az = 40; s.accel_mag = 40;
      s.gx = s.gy = s.gz = 0;
      s.pressure_pa = 101325;
      s.altitude_m = 100;
      s.vert_vel_ms = 20;
      s.imu_health = SensorHealth::OK;
      s.baro_health = SensorHealth::OK;
      Sensors::injectSim(s);
      FlightDetect::injectPhase(FlightPhase::LIFTOFF);
      MissionFsm::setFlightPhase(FlightPhase::LIFTOFF, "ground_test");
    } else if (!strcmp(scenario, "landed")) {
      FlightDetect::injectPhase(FlightPhase::LANDED);
      MissionFsm::setFlightPhase(FlightPhase::LANDED, "ground_test");
    } else if (!strcmp(scenario, "off")) {
      Sensors::setSimulation(false);
    }
    result = CmdResult::ACK;
    detail = "ground_test";
  } else if (!strcmp(cmd, Cmd::LIST_FLIGHTS) || !strcmp(cmd, "list_flights")) {
    Storage::FlightInfo flights[Storage::MAX_FLIGHT_LIST];
    const int n = Storage::listFlights(flights, Storage::MAX_FLIGHT_LIST);
    // Build compact list JSON and send on event channel (may be larger than ACK).
    char list[900];
    size_t pos = 0;
    pos += snprintf(list + pos, sizeof(list) - pos,
                    "{\"v\":%d,\"cmd\":\"list_flights\",\"result\":\"ACK\",\"seq\":%lu,\"flights\":[",
                    PROTOCOL_VERSION_MAJOR, static_cast<unsigned long>(seq));
    for (int i = 0; i < n && pos + 80 < sizeof(list); i++) {
      pos += snprintf(list + pos, sizeof(list) - pos,
                      "%s{\"name\":\"%s\",\"bytes\":%lu,\"files\":%u,\"incomplete\":%s,\"active\":%s}",
                      i ? "," : "",
                      flights[i].name,
                      static_cast<unsigned long>(flights[i].total_bytes),
                      static_cast<unsigned>(flights[i].file_count),
                      flights[i].incomplete ? "true" : "false",
                      flights[i].is_active ? "true" : "false");
    }
    snprintf(list + pos, sizeof(list) - pos, "]}");
    BleServer::notifyEventJson(list);
    result = CmdResult::ACK;
    detail = "list_flights";
    if (response_out && response_len) {
      snprintf(response_out, response_len,
               "{\"v\":%d,\"cmd\":\"list_flights\",\"result\":\"ACK\",\"count\":%d,\"seq\":%lu}",
               PROTOCOL_VERSION_MAJOR, n, static_cast<unsigned long>(seq));
    }
    EventLog::emitCommand(cmd, msg_id, body, result, detail);
    return result;
  } else if (!strcmp(cmd, Cmd::LIST_FILES) || !strcmp(cmd, "list_files")) {
    char flight[48] = "";
    jsonString(body, "flight", flight, sizeof(flight));
    Storage::FileInfo files[Storage::MAX_FILE_LIST];
    const int n = Storage::listFiles(flight, files, Storage::MAX_FILE_LIST);
    char list[700];
    size_t pos = 0;
    pos += snprintf(list + pos, sizeof(list) - pos,
                    "{\"v\":%d,\"cmd\":\"list_files\",\"result\":\"ACK\",\"seq\":%lu,\"flight\":\"%s\",\"files\":[",
                    PROTOCOL_VERSION_MAJOR, static_cast<unsigned long>(seq), flight);
    for (int i = 0; i < n && pos + 60 < sizeof(list); i++) {
      pos += snprintf(list + pos, sizeof(list) - pos,
                      "%s{\"name\":\"%s\",\"size\":%lu}",
                      i ? "," : "", files[i].name, static_cast<unsigned long>(files[i].size));
    }
    snprintf(list + pos, sizeof(list) - pos, "]}");
    BleServer::notifyEventJson(list);
    result = n >= 0 ? CmdResult::ACK : CmdResult::NACK;
    detail = "list_files";
    if (response_out && response_len) {
      snprintf(response_out, response_len,
               "{\"v\":%d,\"cmd\":\"list_files\",\"result\":\"%s\",\"count\":%d,\"seq\":%lu}",
               PROTOCOL_VERSION_MAJOR, result == CmdResult::ACK ? "ACK" : "NACK", n,
               static_cast<unsigned long>(seq));
    }
    EventLog::emitCommand(cmd, msg_id, body, result, detail);
    return result;
  } else if (!strcmp(cmd, Cmd::FILE_BEGIN) || !strcmp(cmd, "file_begin")) {
    char flight[48] = "";
    char file[40] = "";
    jsonString(body, "flight", flight, sizeof(flight));
    jsonString(body, "file", file, sizeof(file));
    uint32_t size = 0;
    if (!Storage::beginFileRead(flight, file, &size)) {
      result = CmdResult::NACK;
      detail = MissionFsm::recordingActive() ? "recording_active_or_missing" : "open_failed";
    } else {
      result = CmdResult::ACK;
      detail = "file_open";
      if (response_out && response_len) {
        snprintf(response_out, response_len,
                 "{\"v\":%d,\"cmd\":\"file_begin\",\"result\":\"ACK\",\"flight\":\"%s\","
                 "\"file\":\"%s\",\"size\":%lu,\"chunk\":%u,\"seq\":%lu}",
                 PROTOCOL_VERSION_MAJOR, flight, file,
                 static_cast<unsigned long>(size),
                 static_cast<unsigned>(BLE_FILE_CHUNK_MAX),
                 static_cast<unsigned long>(seq));
        EventLog::emitCommand(cmd, msg_id, body, result, detail);
        return result;
      }
    }
  } else if (!strcmp(cmd, Cmd::FILE_READ) || !strcmp(cmd, "file_read")) {
    if (!Storage::fileReadOpen()) {
      result = CmdResult::NACK;
      detail = "no_open_file";
    } else {
      const uint32_t offset = static_cast<uint32_t>(jsonInt64(body, "offset", 0));
      int req = jsonInt(body, "len", static_cast<int>(BLE_FILE_CHUNK_MAX));
      if (req <= 0) req = static_cast<int>(BLE_FILE_CHUNK_MAX);
      if (req > static_cast<int>(BLE_FILE_CHUNK_MAX)) req = static_cast<int>(BLE_FILE_CHUNK_MAX);
      const uint32_t total = Storage::fileReadSize();
      uint8_t buf[BLE_FILE_CHUNK_MAX];
      size_t got = Storage::readFileAt(offset, buf, static_cast<size_t>(req));
      if (got == 0) {
        result = CmdResult::ACK;
        detail = (offset >= total) ? "eof" : "read_fail";
        if (response_out && response_len) {
          snprintf(response_out, response_len,
                   "{\"v\":%d,\"cmd\":\"file_read\",\"result\":\"ACK\",\"offset\":%lu,\"len\":0,"
                   "\"eof\":true,\"seq\":%lu}",
                   PROTOCOL_VERSION_MAJOR, static_cast<unsigned long>(offset),
                   static_cast<unsigned long>(seq));
          EventLog::emitCommand(cmd, msg_id, body, result, detail);
          return result;
        }
      } else {
        BleServer::notifyFileChunk(offset, total, buf, static_cast<uint16_t>(got));
        result = CmdResult::ACK;
        detail = "chunk";
        if (response_out && response_len) {
          snprintf(response_out, response_len,
                   "{\"v\":%d,\"cmd\":\"file_read\",\"result\":\"ACK\",\"offset\":%lu,\"len\":%u,"
                   "\"total\":%lu,\"seq\":%lu}",
                   PROTOCOL_VERSION_MAJOR,
                   static_cast<unsigned long>(offset),
                   static_cast<unsigned>(got),
                   static_cast<unsigned long>(total),
                   static_cast<unsigned long>(seq));
          EventLog::emitCommand(cmd, msg_id, body, result, detail);
          return result;
        }
      }
    }
  } else if (!strcmp(cmd, Cmd::FILE_CLOSE) || !strcmp(cmd, "file_close")) {
    Storage::endFileRead();
    result = CmdResult::ACK;
    detail = "closed";
  } else if (!strcmp(cmd, Cmd::DELETE_FLIGHT) || !strcmp(cmd, "delete_flight")) {
    char flight[48] = "";
    jsonString(body, "flight", flight, sizeof(flight));
    if (MissionFsm::recordingActive()) {
      result = CmdResult::REJECTED;
      detail = "recording_active";
    } else if (!Storage::deleteFlight(flight)) {
      result = CmdResult::NACK;
      detail = "delete_failed";
    } else {
      result = CmdResult::ACK;
      detail = "deleted";
    }
  } else {
    result = CmdResult::NACK;
    detail = "unknown_cmd";
  }

  if (critical && (result == CmdResult::ACK || result == CmdResult::IGNORED)) {
    rememberProcessed(msg_id);
  }

  EventLog::emitCommand(cmd, msg_id, body, result, detail);
  writeResponse(response_out, response_len, cmd, result, detail, seq);
  return result;
}

}  // namespace CommandHandler
