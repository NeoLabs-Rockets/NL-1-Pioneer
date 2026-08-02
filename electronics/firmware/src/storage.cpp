#include "storage.h"
#include "config.h"
#include "board_pins.h"
#include "event_log.h"
#include "time_manager.h"
#include "crc16.h"
#include "mission_fsm.h"

#ifndef UNIT_TEST
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <FS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esp_system.h>
#endif

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <ctype.h>

namespace Storage {
namespace {

StorageState st = StorageState::UNKNOWN;
uint64_t free_b = 0;
uint64_t total_b = 0;

bool session_open = false;
char session_dir[96] = "";
char path_telem[112] = "";
char path_events[112] = "";
char path_video[112] = "";
char path_meta[112] = "";
char path_errors[112] = "";

#ifndef UNIT_TEST
File f_telem;
File f_events;
File f_video;
File f_meta;
SPIClass sdSpi(FSPI);

struct FrameMsg {
  uint32_t len;
  uint32_t index;
  uint64_t mono_us;
  // payload follows in separate pool — keep small messages: store ptr into PSRAM pool
  uint8_t* data;
};

QueueHandle_t telem_q = nullptr;
QueueHandle_t frame_q = nullptr;
#endif

uint32_t session_seq = 0;
uint32_t frames_written = 0;
uint32_t frames_dropped = 0;
float avg_fps = 0;
int cam_w = 0, cam_h = 0, cam_fps = 0, cam_q = 0, cam_profile = 0;
char start_reason[40] = "";
uint32_t boot_session_counter = 0;
uint64_t last_flush_ms = 0;

void refreshSpace() {
#ifndef UNIT_TEST
  total_b = SD.totalBytes();
  free_b = SD.usedBytes() <= total_b ? (total_b - SD.usedBytes()) : 0;
#endif
}

bool ensureDir(const char* path) {
#ifndef UNIT_TEST
  if (SD.exists(path)) return true;
  return SD.mkdir(path);
#else
  (void)path;
  return true;
#endif
}

void writeMetadataJson(bool incomplete) {
#ifndef UNIT_TEST
  if (!session_open) return;
  File f = SD.open(path_meta, FILE_WRITE);
  if (!f) return;
  TimeStamp ts = TimeManager::now();
  f.printf("{\n");
  f.printf("  \"firmware_version\": \"%s\",\n", FIRMWARE_VERSION);
  f.printf("  \"board\": \"%s\",\n", BOARD_MODEL_NAME);
  f.printf("  \"camera\": \"%s\",\n", CAMERA_MODEL_NAME);
  f.printf("  \"protocol\": \"%d.%d\",\n", PROTOCOL_VERSION_MAJOR, PROTOCOL_VERSION_MINOR);
  f.printf("  \"session_path\": \"%s\",\n", session_dir);
  f.printf("  \"start_reason\": \"%s\",\n", start_reason);
  f.printf("  \"incomplete\": %s,\n", incomplete ? "true" : "false");
  f.printf("  \"mono_us_close\": %llu,\n", static_cast<unsigned long long>(ts.mono_us));
  f.printf("  \"unix_ms_close\": %lld,\n", static_cast<long long>(ts.sync_unix_ms));
  f.printf("  \"sensors\": {\"imu\": \"LSM6DSO32\", \"baro\": \"BMP580\", \"i2c_sda\": %d, \"i2c_scl\": %d},\n",
           PIN_I2C_SDA, PIN_I2C_SCL);
  f.printf("  \"camera_settings\": {\"w\": %d, \"h\": %d, \"fps\": %d, \"jpeg_q\": %d, \"profile\": %d},\n",
           cam_w, cam_h, cam_fps, cam_q, cam_profile);
  f.printf("  \"frames_written\": %lu,\n", static_cast<unsigned long>(frames_written));
  f.printf("  \"frames_dropped\": %lu,\n", static_cast<unsigned long>(frames_dropped));
  f.printf("  \"avg_fps\": %.2f,\n", avg_fps);
  f.printf("  \"mission_state\": \"%s\",\n", missionStateName(MissionFsm::state()));
  f.printf("  \"flight_phase\": \"%s\",\n", flightPhaseName(MissionFsm::flightPhase()));
  f.printf("  \"expected_ignition_mono_ms\": %lld,\n",
           static_cast<long long>(MissionFsm::expectedIgnitionMonoMs()));
  f.printf("  \"actual_liftoff_mono_ms\": %lld,\n",
           static_cast<long long>(MissionFsm::actualLiftoffMonoMs()));
  f.printf("  \"countdown_start_mono_ms\": %lld,\n",
           static_cast<long long>(MissionFsm::countdownStartMonoMs()));
  f.printf("  \"countdown_duration_ms\": %lu,\n",
           static_cast<unsigned long>(MissionFsm::countdownDurationMs()));
  if (MissionFsm::expectedIgnitionMonoMs() >= 0 && MissionFsm::actualLiftoffMonoMs() >= 0) {
    f.printf("  \"liftoff_delta_ms\": %lld,\n",
             static_cast<long long>(MissionFsm::actualLiftoffMonoMs()
                                    - MissionFsm::expectedIgnitionMonoMs()));
  }
  f.printf("  \"free_sd_bytes\": %llu,\n", static_cast<unsigned long long>(free_b));
  f.printf("  \"max_mission_record_ms\": %lu,\n", static_cast<unsigned long>(MAX_MISSION_RECORD_MS));
  f.printf("  \"post_landing_record_ms\": %lu\n", static_cast<unsigned long>(POST_LANDING_RECORD_MS));
  f.printf("}\n");
  f.close();
#else
  (void)incomplete;
#endif
}

// Minimal MJPEG AVI writer (Motion-JPEG)
// We write a simplified AVI that common players accept for MJPEG streams.
struct AviState {
  uint32_t frames = 0;
  uint32_t max_frame = 0;
  uint32_t movi_size = 4;  // 'movi' list payload starts after 4-char
  bool header_written = false;
  uint32_t fps = 12;
  uint32_t width = 800;
  uint32_t height = 600;
} avi;

void writeAviHeaderPlaceholder() {
#ifndef UNIT_TEST
  // Write oversized placeholder; finalized on close with known sizes when possible.
  // For crash resilience we use a "open DML" style with large sizes.
  uint8_t hdr[512];
  memset(hdr, 0, sizeof(hdr));
  // RIFF size patched later — start with zeros and rewrite on close if possible.
  // For simplicity write raw MJPEG concatenation as .mjpg companion is more robust;
  // requirement allows AVI or other ESP32-friendly container.
  // We write: [u32 mono_us][u32 len][jpeg bytes]... as video.mjpg
  // AND a lightweight video.avi index-less stream if needed.
  // Primary container: video.mjpg (length-prefixed JPEG sequence) — documented.
  (void)hdr;
  avi.header_written = true;
#endif
}

}  // namespace

bool begin() {
  st = StorageState::UNKNOWN;
  session_open = false;
#ifndef UNIT_TEST
  sdSpi.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, sdSpi, 25000000)) {
    st = StorageState::MISSING;
    EventLog::emit(EventType::STORAGE_FAULT, "sd_mount_failed");
    return false;
  }
  refreshSpace();
  st = free_b < SD_MIN_FREE_BYTES ? StorageState::FULL : StorageState::MOUNTED;
  ensureDir(FLIGHTS_DIR);
  telem_q = xQueueCreate(TELEMETRY_RING_SAMPLES, sizeof(TelemetryRecord));
  frame_q = xQueueCreate(FRAME_QUEUE_LEN, sizeof(FrameMsg));
  recoverOrphanSessions();
  return true;
#else
  st = StorageState::MOUNTED;
  free_b = 64ULL * 1024 * 1024;
  return true;
#endif
}

StorageState state() { return st; }
uint64_t freeBytes() { return free_b; }
uint64_t totalBytes() { return total_b; }

void recoverOrphanSessions() {
#ifndef UNIT_TEST
  // Mark any session dirs that still have incomplete.flag
  File root = SD.open(FLIGHTS_DIR);
  if (!root || !root.isDirectory()) return;
  File e = root.openNextFile();
  while (e) {
    if (e.isDirectory()) {
      char flag[128];
      snprintf(flag, sizeof(flag), "%s/incomplete.flag", e.path());
      if (SD.exists(flag)) {
        // leave flag; metadata already says incomplete
      }
    }
    e = root.openNextFile();
  }
#endif
}

bool openSession(const char* reason) {
  if (session_open) return true;
  if (st == StorageState::MISSING || st == StorageState::ERROR) return false;
  refreshSpace();
  if (free_b < SD_MIN_FREE_BYTES) {
    st = StorageState::FULL;
    EventLog::emit(EventType::STORAGE_FAULT, "sd_full");
    return false;
  }

  boot_session_counter++;
  TimeStamp ts = TimeManager::now();
  if (ts.sync_unix_ms > 0) {
    // flight_YYYYMMDD_HHMMSS — approximate from unix ms
    time_t sec = static_cast<time_t>(ts.sync_unix_ms / 1000);
    struct tm tmv;
    gmtime_r(&sec, &tmv);
    snprintf(session_dir, sizeof(session_dir),
             "%s/flight_%04d%02d%02d_%02d%02d%02d",
             FLIGHTS_DIR, tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
  } else {
    snprintf(session_dir, sizeof(session_dir), "%s/flight_boot_%04lu",
             FLIGHTS_DIR, static_cast<unsigned long>(boot_session_counter));
  }
  // Avoid collisions
#ifndef UNIT_TEST
  if (SD.exists(session_dir)) {
    char alt[96];
    snprintf(alt, sizeof(alt), "%s_%lu", session_dir, static_cast<unsigned long>(ts.mono_us % 10000));
    strncpy(session_dir, alt, sizeof(session_dir) - 1);
  }
  if (!ensureDir(session_dir)) {
    st = StorageState::ERROR;
    EventLog::emit(EventType::STORAGE_FAULT, "mkdir_failed");
    return false;
  }
  snprintf(path_telem, sizeof(path_telem), "%s/telemetry.bin", session_dir);
  snprintf(path_events, sizeof(path_events), "%s/events.jsonl", session_dir);
  snprintf(path_video, sizeof(path_video), "%s/video.mjpg", session_dir);
  snprintf(path_meta, sizeof(path_meta), "%s/metadata.json", session_dir);
  snprintf(path_errors, sizeof(path_errors), "%s/errors.log", session_dir);

  f_telem = SD.open(path_telem, FILE_WRITE);
  f_events = SD.open(path_events, FILE_WRITE);
  f_video = SD.open(path_video, FILE_WRITE);
  if (!f_telem || !f_events || !f_video) {
    st = StorageState::ERROR;
    EventLog::emit(EventType::STORAGE_FAULT, "file_open_failed");
    return false;
  }
  // incomplete flag
  char flag[112];
  snprintf(flag, sizeof(flag), "%s/incomplete.flag", session_dir);
  File fl = SD.open(flag, FILE_WRITE);
  if (fl) {
    fl.println("1");
    fl.close();
  }
#endif

  strncpy(start_reason, reason ? reason : "unknown", sizeof(start_reason) - 1);
  frames_written = 0;
  frames_dropped = 0;
  session_seq = 0;
  session_open = true;
  st = StorageState::WRITING;
  TimeManager::startSession();
  writeAviHeaderPlaceholder();
  writeMetadataJson(true);
  EventLog::emit(EventType::RECORDING_START, session_dir);
  return true;
}

bool isSessionOpen() { return session_open; }
const char* sessionPath() { return session_dir; }

void writeTelemetry(const TelemetryRecord& rec) {
#ifndef UNIT_TEST
  if (!session_open || !f_telem) return;
  f_telem.write(reinterpret_cast<const uint8_t*>(&rec), sizeof(rec));
#else
  (void)rec;
#endif
}

void writeVideoFrame(const uint8_t* jpeg, size_t len, uint32_t frame_index, uint64_t mono_us) {
#ifndef UNIT_TEST
  if (!session_open || !f_video || !jpeg || !len) return;
  // Length-prefixed MJPEG sequence: [u64 mono_us][u32 index][u32 len][jpeg]
  f_video.write(reinterpret_cast<const uint8_t*>(&mono_us), sizeof(mono_us));
  f_video.write(reinterpret_cast<const uint8_t*>(&frame_index), sizeof(frame_index));
  uint32_t l = static_cast<uint32_t>(len);
  f_video.write(reinterpret_cast<const uint8_t*>(&l), sizeof(l));
  f_video.write(jpeg, len);
  frames_written++;
  avi.frames++;
  if (len > avi.max_frame) avi.max_frame = len;
#else
  (void)jpeg; (void)len; (void)frame_index; (void)mono_us;
#endif
}

void flush() {
#ifndef UNIT_TEST
  if (f_telem) f_telem.flush();
  if (f_events) f_events.flush();
  if (f_video) f_video.flush();
  last_flush_ms = TimeManager::monoMs();
  refreshSpace();
#endif
}

void closeSession(const char* reason) {
  if (!session_open) return;
  serviceEvents();
  flush();
  writeMetadataJson(false);
#ifndef UNIT_TEST
  if (f_telem) f_telem.close();
  if (f_events) f_events.close();
  if (f_video) f_video.close();
  char flag[112];
  snprintf(flag, sizeof(flag), "%s/incomplete.flag", session_dir);
  SD.remove(flag);
#endif
  session_open = false;
  st = free_b < SD_MIN_FREE_BYTES ? StorageState::FULL : StorageState::MOUNTED;
  EventLog::emit(EventType::RECORDING_STOP, reason ? reason : "close");
}

void serviceEvents() {
#ifndef UNIT_TEST
  if (!session_open || !f_events) return;
  char line[384];
  while (EventLog::pop(line, sizeof(line))) {
    f_events.print(line);
  }
#endif
}

void noteEventForMetadata(const char* key, const char* value) {
  (void)key; (void)value;
}

void setCameraMeta(int width, int height, int fps, int quality, int profile) {
  cam_w = width; cam_h = height; cam_fps = fps; cam_q = quality; cam_profile = profile;
  avi.width = width; avi.height = height; avi.fps = fps > 0 ? fps : 10;
}

void setFrameStats(uint32_t written, uint32_t dropped, float avg) {
  frames_written = written;
  frames_dropped = dropped;
  avg_fps = avg;
}

bool enqueueTelemetry(const TelemetryRecord& rec) {
#ifndef UNIT_TEST
  if (!telem_q) return false;
  return xQueueSend(telem_q, &rec, 0) == pdTRUE;
#else
  (void)rec;
  return true;
#endif
}

bool enqueueFrame(const uint8_t* data, size_t len, uint32_t idx, uint64_t mono_us) {
#ifndef UNIT_TEST
  if (!frame_q || !data || !len) return false;
  uint8_t* copy = static_cast<uint8_t*>(ps_malloc(len));
  if (!copy) {
    frames_dropped++;
    return false;
  }
  memcpy(copy, data, len);
  FrameMsg m{static_cast<uint32_t>(len), idx, mono_us, copy};
  if (xQueueSend(frame_q, &m, 0) != pdTRUE) {
    free(copy);
    frames_dropped++;
    return false;
  }
  return true;
#else
  (void)data; (void)len; (void)idx; (void)mono_us;
  return true;
#endif
}

void taskLoop() {
#ifndef UNIT_TEST
  TelemetryRecord tr;
  FrameMsg fm;
  if (telem_q) {
    while (xQueueReceive(telem_q, &tr, 0) == pdTRUE) {
      writeTelemetry(tr);
    }
  }
  if (frame_q) {
    while (xQueueReceive(frame_q, &fm, 0) == pdTRUE) {
      writeVideoFrame(fm.data, fm.len, fm.index, fm.mono_us);
      free(fm.data);
    }
  }
  serviceEvents();
  uint64_t now = TimeManager::monoMs();
  if (session_open && now - last_flush_ms >= SD_FLUSH_INTERVAL_MS) flush();
#endif
}

// ── Flight export ───────────────────────────────────────────────────────────
namespace {
#ifndef UNIT_TEST
File f_read;
char read_path[128] = "";
uint32_t read_size = 0;
bool read_open = false;

bool safeFlightName(const char* name) {
  if (!name || !name[0] || strlen(name) >= 48) return false;
  if (strstr(name, "..")) return false;
  for (const char* p = name; *p; p++) {
    const char c = *p;
    if (c == '/' || c == '\\') return false;
    if (!(isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == '.')) return false;
  }
  if (name[0] == '.') return false;
  return true;
}

bool safeFileName(const char* name) {
  if (!name || !name[0] || strlen(name) >= 40) return false;
  for (const char* p = name; *p; p++) {
    const char c = *p;
    if (c == '/' || c == '\\') return false;
    if (!(isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == '.')) return false;
  }
  if (name[0] == '.') return false;
  return true;
}

bool removeTree(const char* path) {
  File dir = SD.open(path);
  if (!dir) return false;
  if (!dir.isDirectory()) {
    dir.close();
    return SD.remove(path);
  }
  File e = dir.openNextFile();
  while (e) {
    char child[128];
    // e.name() may be full path or basename depending on FS
    const char* n = e.name();
    if (strchr(n, '/')) {
      strncpy(child, n, sizeof(child) - 1);
      child[sizeof(child) - 1] = '\0';
    } else {
      snprintf(child, sizeof(child), "%s/%s", path, n);
    }
    const bool isDir = e.isDirectory();
    e.close();
    if (isDir) {
      if (!removeTree(child)) {
        dir.close();
        return false;
      }
    } else if (!SD.remove(child)) {
      dir.close();
      return false;
    }
    e = dir.openNextFile();
  }
  dir.close();
  return SD.rmdir(path);
}
#endif
}  // namespace

bool readyForTransfer() {
  return st == StorageState::MOUNTED || st == StorageState::WRITING || st == StorageState::FULL;
}

int listFlights(FlightInfo* out, int max) {
  if (!out || max <= 0) return 0;
  int count = 0;
#ifndef UNIT_TEST
  if (!readyForTransfer()) return 0;
  File root = SD.open(FLIGHTS_DIR);
  if (!root || !root.isDirectory()) return 0;
  File e = root.openNextFile();
  while (e && count < max) {
    if (e.isDirectory()) {
      const char* raw = e.name();
      const char* base = strrchr(raw, '/');
      base = base ? base + 1 : raw;
      if (base[0] && base[0] != '.') {
        FlightInfo& fi = out[count];
        memset(&fi, 0, sizeof(fi));
        strncpy(fi.name, base, sizeof(fi.name) - 1);
        fi.is_active = session_open && strstr(session_dir, base) != nullptr;
        char flag[128];
        snprintf(flag, sizeof(flag), "%s/%s/incomplete.flag", FLIGHTS_DIR, base);
        fi.incomplete = SD.exists(flag);
        // Sum file sizes
        char dirPath[96];
        snprintf(dirPath, sizeof(dirPath), "%s/%s", FLIGHTS_DIR, base);
        File d = SD.open(dirPath);
        if (d && d.isDirectory()) {
          File f = d.openNextFile();
          while (f) {
            if (!f.isDirectory()) {
              fi.total_bytes += static_cast<uint32_t>(f.size());
              fi.file_count++;
            }
            f = d.openNextFile();
          }
          d.close();
        }
        count++;
      }
    }
    e = root.openNextFile();
  }
  root.close();
#else
  (void)out; (void)max;
#endif
  return count;
}

int listFiles(const char* flightName, FileInfo* out, int max) {
  if (!out || max <= 0) return 0;
  int count = 0;
#ifndef UNIT_TEST
  if (!readyForTransfer() || !safeFlightName(flightName)) return 0;
  char dirPath[96];
  snprintf(dirPath, sizeof(dirPath), "%s/%s", FLIGHTS_DIR, flightName);
  File d = SD.open(dirPath);
  if (!d || !d.isDirectory()) return 0;
  File f = d.openNextFile();
  while (f && count < max) {
    if (!f.isDirectory()) {
      const char* raw = f.name();
      const char* base = strrchr(raw, '/');
      base = base ? base + 1 : raw;
      if (strcmp(base, "incomplete.flag") != 0) {
        FileInfo& fi = out[count];
        memset(&fi, 0, sizeof(fi));
        strncpy(fi.name, base, sizeof(fi.name) - 1);
        fi.size = static_cast<uint32_t>(f.size());
        count++;
      }
    }
    f = d.openNextFile();
  }
  d.close();
#else
  (void)flightName;
#endif
  return count;
}

bool beginFileRead(const char* flightName, const char* fileName, uint32_t* sizeOut) {
#ifndef UNIT_TEST
  endFileRead();
  if (!readyForTransfer() || !safeFlightName(flightName) || !safeFileName(fileName)) return false;
  if (session_open && strstr(session_dir, flightName)) {
    // Allow read of closed files only — refuse active session export mid-record
    return false;
  }
  snprintf(read_path, sizeof(read_path), "%s/%s/%s", FLIGHTS_DIR, flightName, fileName);
  f_read = SD.open(read_path, FILE_READ);
  if (!f_read) {
    read_path[0] = '\0';
    return false;
  }
  read_size = static_cast<uint32_t>(f_read.size());
  read_open = true;
  if (sizeOut) *sizeOut = read_size;
  return true;
#else
  (void)flightName; (void)fileName; (void)sizeOut;
  return false;
#endif
}

size_t readFileAt(uint32_t offset, uint8_t* buf, size_t maxLen) {
#ifndef UNIT_TEST
  if (!read_open || !f_read || !buf || !maxLen) return 0;
  if (offset >= read_size) return 0;
  if (!f_read.seek(offset)) return 0;
  size_t want = maxLen;
  if (offset + want > read_size) want = read_size - offset;
  return f_read.read(buf, want);
#else
  (void)offset; (void)buf; (void)maxLen;
  return 0;
#endif
}

void endFileRead() {
#ifndef UNIT_TEST
  if (f_read) f_read.close();
  read_open = false;
  read_size = 0;
  read_path[0] = '\0';
#endif
}

bool fileReadOpen() {
#ifndef UNIT_TEST
  return read_open;
#else
  return false;
#endif
}

const char* fileReadPath() {
#ifndef UNIT_TEST
  return read_path;
#else
  return "";
#endif
}

uint32_t fileReadSize() {
#ifndef UNIT_TEST
  return read_size;
#else
  return 0;
#endif
}

bool deleteFlight(const char* flightName) {
#ifndef UNIT_TEST
  if (!readyForTransfer() || !safeFlightName(flightName)) return false;
  if (session_open && strstr(session_dir, flightName)) return false;
  if (read_open && strstr(read_path, flightName)) endFileRead();
  char dirPath[96];
  snprintf(dirPath, sizeof(dirPath), "%s/%s", FLIGHTS_DIR, flightName);
  if (!SD.exists(dirPath)) return false;
  const bool ok = removeTree(dirPath);
  if (ok) {
    refreshSpace();
    EventLog::emit(EventType::WARNING, "flight_deleted", flightName);
  }
  return ok;
#else
  (void)flightName;
  return false;
#endif
}

}  // namespace Storage
