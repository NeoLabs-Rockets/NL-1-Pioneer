#include "time_manager.h"

#ifndef UNIT_TEST
#include <Arduino.h>
#include <esp_timer.h>
#endif

namespace TimeManager {
namespace {

uint64_t session_origin_us = 0;
bool session_on = false;

// sync_unix_ms ≈ mono_ms + offset_ms
int64_t sync_offset_ms = 0;
bool synced = false;
uint8_t quality = 1;  // boot-only
uint32_t last_rtt_ms = 0;

#ifdef UNIT_TEST
uint64_t fake_mono_us = 0;
#endif

}  // namespace

void begin() {
  session_on = false;
  session_origin_us = 0;
  synced = false;
  quality = 1;
  sync_offset_ms = 0;
}

uint64_t monoUs() {
#ifdef UNIT_TEST
  return fake_mono_us;
#else
  return static_cast<uint64_t>(esp_timer_get_time());
#endif
}

uint64_t monoMs() { return monoUs() / 1000ULL; }

TimeStamp now() {
  TimeStamp t{};
  t.mono_us = monoUs();
  t.session_us = session_on && t.mono_us >= session_origin_us
                     ? (t.mono_us - session_origin_us)
                     : 0;
  t.sync_unix_ms = synced ? (static_cast<int64_t>(t.mono_us / 1000ULL) + sync_offset_ms) : 0;
  t.time_quality = quality;
  return t;
}

void startSession(uint64_t mono_us) {
  session_origin_us = mono_us ? mono_us : monoUs();
  session_on = true;
}

void endSession() { session_on = false; }

bool sessionActive() { return session_on; }

uint64_t sessionUs() {
  if (!session_on) return 0;
  const uint64_t m = monoUs();
  return m >= session_origin_us ? (m - session_origin_us) : 0;
}

void applySync(int64_t remote_unix_ms, uint32_t rtt_ms, uint64_t local_mono_ms_at_rx) {
  // Compensate one-way latency as half RTT when available.
  const int64_t one_way = static_cast<int64_t>(rtt_ms / 2);
  const int64_t corrected_remote = remote_unix_ms + one_way;
  sync_offset_ms = corrected_remote - static_cast<int64_t>(local_mono_ms_at_rx);
  synced = true;
  last_rtt_ms = rtt_ms;
  quality = rtt_ms > 0 ? 3 : 2;
}

bool isSynced() { return synced; }
uint8_t timeQuality() { return quality; }

int64_t syncUnixMs() {
  if (!synced) return 0;
  return static_cast<int64_t>(monoMs()) + sync_offset_ms;
}

int64_t monoMsToUnixMs(uint64_t mono_ms) {
  if (!synced) return 0;
  return static_cast<int64_t>(mono_ms) + sync_offset_ms;
}

#ifdef UNIT_TEST
void setFakeMonoUs(uint64_t us) { fake_mono_us = us; }
#endif

}  // namespace TimeManager
