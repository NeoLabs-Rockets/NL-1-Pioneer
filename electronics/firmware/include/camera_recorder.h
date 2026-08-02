#pragma once
#include "types.h"
#include "config.h"
#include <stdint.h>

namespace CameraRecorder {

struct Stats {
  uint32_t frames_written;
  uint32_t frames_dropped;
  float actual_fps;
  CameraProfileId profile;
  int width;
  int height;
  int jpeg_quality;
  int target_fps;
  size_t psram_free;
  size_t psram_size;
  float expected_mbit_s;
};

bool begin();  // init sensor, select highest stable profile
void end();

bool start();  // begin capture loop contribution
void stop();

bool isRecording();
CameraState state();
Stats stats();

// Called from camera task — grab frame and enqueue to storage.
void service();

// Force profile fallback (e.g. after repeated drops).
bool fallbackProfile();

}  // namespace CameraRecorder
