#pragma once
#include "types.h"
#include <stdint.h>
#include <stddef.h>

namespace Storage {

bool begin();
StorageState state();
uint64_t freeBytes();
uint64_t totalBytes();

// Session lifecycle — single writer for SD card.
bool openSession(const char* reason);
bool isSessionOpen();
const char* sessionPath();
void writeTelemetry(const TelemetryRecord& rec);
void writeVideoFrame(const uint8_t* jpeg, size_t len, uint32_t frame_index, uint64_t mono_us);
void flush();
void closeSession(const char* reason);  // flush + finalize metadata

// Mark incomplete sessions after crash recovery.
void recoverOrphanSessions();

// Drain event queue to events.jsonl
void serviceEvents();

// Metadata helpers
void noteEventForMetadata(const char* key, const char* value);
void setCameraMeta(int width, int height, int fps, int quality, int profile);
void setFrameStats(uint32_t written, uint32_t dropped, float avg_fps);

// Enqueue telemetry from sensor task (ISR-safe-ish via FreeRTOS queue).
bool enqueueTelemetry(const TelemetryRecord& rec);
bool enqueueFrame(const uint8_t* data, size_t len, uint32_t idx, uint64_t mono_us);

// Storage task entry
void taskLoop();

// ── Flight data export (BLE download) ───────────────────────────────────────
static constexpr int MAX_FLIGHT_LIST = 24;
static constexpr int MAX_FILE_LIST = 12;

struct FlightInfo {
  char name[48];
  uint32_t total_bytes;
  uint8_t file_count;
  bool incomplete;
  bool is_active;  // currently open recording session
};

struct FileInfo {
  char name[40];
  uint32_t size;
};

// Returns number of flights written to out (<= max).
int listFlights(FlightInfo* out, int max);

// List files inside /flights/<flightName>/
int listFiles(const char* flightName, FileInfo* out, int max);

// Open a file for sequential/random read. Fails if active recording uses it.
bool beginFileRead(const char* flightName, const char* fileName, uint32_t* sizeOut);
// Read up to maxLen bytes at absolute offset. Returns bytes read (0 = EOF/error).
size_t readFileAt(uint32_t offset, uint8_t* buf, size_t maxLen);
void endFileRead();
bool fileReadOpen();
const char* fileReadPath();
uint32_t fileReadSize();

// Delete an entire flight directory. Refuses active session.
bool deleteFlight(const char* flightName);

// True if SD is mounted and not in a failed state.
bool readyForTransfer();

}  // namespace Storage
