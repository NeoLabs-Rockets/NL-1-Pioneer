#include "camera_recorder.h"
#include "board_pins.h"
#include "storage.h"
#include "event_log.h"
#include "time_manager.h"

#ifndef UNIT_TEST
#include <Arduino.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#endif

namespace CameraRecorder {
namespace {

CameraState cam_state = CameraState::UNINIT;
CameraProfileId profile = CameraProfileId::Primary_SVGA_12;
bool recording = false;
Stats st{};

uint32_t frame_idx = 0;
uint32_t window_frames = 0;
uint64_t window_start_us = 0;
uint32_t drop_streak = 0;

struct ProfileDef {
  const char* name;
  int framesize;
  int quality;
  int fps;
  int fb_count;
  int width;
  int height;
};

// framesize ints match framesize_t in esp_camera.h
const ProfileDef kProfiles[] = {
  {"SVGA_12", CAM_FRAMESIZE_SVGA, 12, 12, 2, 800, 600},
  {"VGA_10",  CAM_FRAMESIZE_VGA,  12, 10, 2, 640, 480},
  {"QVGA_10", CAM_FRAMESIZE_QVGA, 12, 10, 1, 320, 240},
};

void updateStatsMeta() {
  const ProfileDef& p = kProfiles[static_cast<int>(profile)];
  st.profile = profile;
  st.width = p.width;
  st.height = p.height;
  st.jpeg_quality = p.quality;
  st.target_fps = p.fps;
  // Rough JPEG size estimate for expected bitrate
  float bytes_per_frame = (p.width * p.height * 0.15f);  // ~0.15 B/px JPEG ballpark
  st.expected_mbit_s = (bytes_per_frame * p.fps * 8.f) / 1e6f;
#ifndef UNIT_TEST
  st.psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  st.psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
#endif
  Storage::setCameraMeta(p.width, p.height, p.fps, p.quality, static_cast<int>(profile));
}

bool initCamera(CameraProfileId id) {
#ifndef UNIT_TEST
  esp_camera_deinit();
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_PIN_D0;
  config.pin_d1 = CAM_PIN_D1;
  config.pin_d2 = CAM_PIN_D2;
  config.pin_d3 = CAM_PIN_D3;
  config.pin_d4 = CAM_PIN_D4;
  config.pin_d5 = CAM_PIN_D5;
  config.pin_d6 = CAM_PIN_D6;
  config.pin_d7 = CAM_PIN_D7;
  config.pin_xclk = CAM_PIN_XCLK;
  config.pin_pclk = CAM_PIN_PCLK;
  config.pin_vsync = CAM_PIN_VSYNC;
  config.pin_href = CAM_PIN_HREF;
  config.pin_sccb_sda = CAM_PIN_SIOD;
  config.pin_sccb_scl = CAM_PIN_SIOC;
  config.pin_pwdn = CAM_PIN_PWDN;
  config.pin_reset = CAM_PIN_RESET;
  config.xclk_freq_hz = 20000000;
  config.frame_size = static_cast<framesize_t>(kProfiles[static_cast<int>(id)].framesize);
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = kProfiles[static_cast<int>(id)].quality;
  config.fb_count = kProfiles[static_cast<int>(id)].fb_count;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[NL1] camera_init profile=%d err=0x%x\n",
                  static_cast<int>(id), static_cast<unsigned>(err));
    EventLog::emit(EventType::CAMERA_FAULT, "camera_init_failed");
    return false;
  }
  Serial.printf("[NL1] camera_init OK profile=%s\n", kProfiles[static_cast<int>(id)].name);
  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor) {
    sensor->set_brightness(sensor, 0);
    sensor->set_contrast(sensor, 0);
    sensor->set_saturation(sensor, 0);
    sensor->set_whitebal(sensor, 1);
    sensor->set_gain_ctrl(sensor, 1);
    sensor->set_exposure_ctrl(sensor, 1);
    sensor->set_vflip(sensor, 0);
    sensor->set_hmirror(sensor, 0);
  }
  profile = id;
  updateStatsMeta();
  return true;
#else
  profile = id;
  updateStatsMeta();
  return true;
#endif
}

}  // namespace

bool begin() {
  cam_state = CameraState::UNINIT;
  recording = false;
  st = {};
  frame_idx = 0;
  // Try primary → fallback → low
  for (int i = 0; i < static_cast<int>(CameraProfileId::Count); i++) {
    if (initCamera(static_cast<CameraProfileId>(i))) {
      cam_state = (i == 0) ? CameraState::READY : CameraState::FALLBACK;
      if (i > 0) EventLog::emit(EventType::WARNING, "camera_fallback_profile");
      return true;
    }
  }
  cam_state = CameraState::FAILED;
  return false;
}

void end() {
#ifndef UNIT_TEST
  esp_camera_deinit();
#endif
  cam_state = CameraState::UNINIT;
  recording = false;
}

bool start() {
  if (cam_state == CameraState::FAILED || cam_state == CameraState::UNINIT) return false;
  recording = true;
  cam_state = CameraState::RECORDING;
  window_frames = 0;
  window_start_us = TimeManager::monoUs();
  drop_streak = 0;
  return true;
}

void stop() {
  recording = false;
  if (cam_state == CameraState::RECORDING)
    cam_state = (profile == CameraProfileId::Primary_SVGA_12) ? CameraState::READY : CameraState::FALLBACK;
  Storage::setFrameStats(st.frames_written, st.frames_dropped, st.actual_fps);
}

bool isRecording() { return recording; }
CameraState state() { return cam_state; }
Stats stats() { return st; }

bool fallbackProfile() {
  int next = static_cast<int>(profile) + 1;
  if (next >= static_cast<int>(CameraProfileId::Count)) return false;
  bool was = recording;
  if (was) stop();
  if (!initCamera(static_cast<CameraProfileId>(next))) {
    cam_state = CameraState::FAILED;
    return false;
  }
  cam_state = CameraState::FALLBACK;
  if (was) start();
  return true;
}

void service() {
  if (!recording) return;
#ifndef UNIT_TEST
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    st.frames_dropped++;
    drop_streak++;
    if (drop_streak > 30) {
      EventLog::emit(EventType::CAMERA_FAULT, "frame_drop_streak");
      fallbackProfile();
      drop_streak = 0;
    }
    return;
  }
  drop_streak = 0;
  uint64_t mono = TimeManager::monoUs();
  bool ok = Storage::enqueueFrame(fb->buf, fb->len, frame_idx++, mono);
  esp_camera_fb_return(fb);
  if (ok) {
    st.frames_written++;
    window_frames++;
  } else {
    st.frames_dropped++;
  }
  uint64_t elapsed = mono - window_start_us;
  if (elapsed >= 1000000ULL) {
    st.actual_fps = window_frames * 1000000.f / static_cast<float>(elapsed);
    window_frames = 0;
    window_start_us = mono;
    // If actual fps falls below 50% of target for sustained period, fallback.
    if (st.actual_fps < st.target_fps * 0.5f && st.frames_written > 60) {
      fallbackProfile();
    }
  }
  updateStatsMeta();
#endif
}

}  // namespace CameraRecorder
