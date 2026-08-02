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

}  // namespace Storage
