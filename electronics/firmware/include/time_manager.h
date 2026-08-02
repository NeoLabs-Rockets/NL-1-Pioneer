#pragma once
#include "types.h"

namespace TimeManager {

void begin();

// Monotonic microseconds since boot (esp_timer / micros64). Never goes backwards.
uint64_t monoUs();
uint64_t monoMs();

// Fill a TimeStamp using current mono time + session + sync state.
TimeStamp now();

// Mark flight-session start (session_us origin).
void startSession(uint64_t mono_us = 0);
void endSession();
bool sessionActive();
uint64_t sessionUs();

// External time sync from dashboard (unix ms + optional RTT estimate).
// offset applied: sync_unix_ms ≈ local_mono_ms + offset
void applySync(int64_t remote_unix_ms, uint32_t rtt_ms, uint64_t local_mono_ms_at_rx);
bool isSynced();
uint8_t timeQuality();
int64_t syncUnixMs();

// Convert mono ms to approx unix ms if synced, else 0.
int64_t monoMsToUnixMs(uint64_t mono_ms);

#ifdef UNIT_TEST
void setFakeMonoUs(uint64_t us);
#endif

}  // namespace TimeManager
