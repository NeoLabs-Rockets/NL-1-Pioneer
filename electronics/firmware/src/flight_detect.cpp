#include "flight_detect.h"
#include "time_manager.h"

#include <math.h>
#include <string.h>

namespace FlightDetect {
namespace {

FlightDetectConfig cfg = kFlightCfg;
FlightPhase current = FlightPhase::PRELAUNCH;

float baseline_alt = NAN;
float peak_agl = 0;
float launch_alt = NAN;

// Rolling quiet confirmation
uint32_t quiet_accum_ms = 0;
uint64_t last_sample_ms = 0;
uint64_t phase_since_ms = 0;
uint64_t condition_since_ms = 0;

// Simple altitude ring for std estimate
static constexpr int ALT_N = 25;
float alt_ring[ALT_N];
int alt_i = 0;
int alt_count = 0;

bool impact_flag = false;
bool forced_phase = false;

float altStd() {
  if (alt_count < 5) return 999.f;
  float mean = 0;
  for (int i = 0; i < alt_count; i++) mean += alt_ring[i];
  mean /= alt_count;
  float var = 0;
  for (int i = 0; i < alt_count; i++) {
    float d = alt_ring[i] - mean;
    var += d * d;
  }
  return sqrtf(var / alt_count);
}

void pushAlt(float a) {
  alt_ring[alt_i] = a;
  alt_i = (alt_i + 1) % ALT_N;
  if (alt_count < ALT_N) alt_count++;
}

void setPhase(FlightPhase p, uint64_t now_ms) {
  if (p == current) return;
  current = p;
  phase_since_ms = now_ms;
  condition_since_ms = now_ms;
  quiet_accum_ms = 0;
}

uint32_t heldMs(uint64_t now_ms, uint64_t since) {
  return now_ms >= since ? static_cast<uint32_t>(now_ms - since) : 0;
}

}  // namespace

void begin(const FlightDetectConfig& c) {
  cfg = c;
  reset();
}

void reset() {
  current = FlightPhase::PRELAUNCH;
  baseline_alt = NAN;
  peak_agl = 0;
  launch_alt = NAN;
  quiet_accum_ms = 0;
  last_sample_ms = 0;
  phase_since_ms = 0;
  condition_since_ms = 0;
  alt_i = 0;
  alt_count = 0;
  impact_flag = false;
  forced_phase = false;
  memset(alt_ring, 0, sizeof(alt_ring));
}

FlightPhase phase() { return current; }
float peakAltitudeAglM() { return peak_agl; }
float baselineAltitudeM() { return baseline_alt; }
bool sawImpact() { return impact_flag; }
uint32_t quietMs() { return quiet_accum_ms; }

void injectPhase(FlightPhase p) {
  forced_phase = true;
  current = p;
}

bool update(const SensorSnapshot& s, int64_t expected_ignition_mono_ms, bool armed_or_countdown) {
  if (forced_phase) return false;
  if (!s.valid) return false;

  const uint64_t now_ms = s.ts.mono_us / 1000ULL;
  const uint32_t dt = last_sample_ms ? static_cast<uint32_t>(now_ms - last_sample_ms) : 10;
  last_sample_ms = now_ms;

  const float amag = s.accel_mag;
  const float gyro_mag = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);
  const float alt = s.altitude_m;
  const float vz = s.vert_vel_ms;

  if (isnan(baseline_alt) && s.baro_health == SensorHealth::OK) {
    baseline_alt = alt;
  }
  if (s.baro_health == SensorHealth::OK) {
    pushAlt(alt);
    if (!isnan(baseline_alt)) {
      float agl = alt - baseline_alt;
      if (agl > peak_agl) peak_agl = agl;
    }
  }

  // Axial heuristic: largest absolute accel axis as "up" estimate pre-launch
  float axial = s.az;
  if (fabsf(s.ax) > fabsf(axial)) axial = s.ax;
  if (fabsf(s.ay) > fabsf(axial)) axial = s.ay;

  FlightPhase prev = current;
  const bool near_ign =
      expected_ignition_mono_ms >= 0 &&
      llabs(static_cast<int64_t>(now_ms) - expected_ignition_mono_ms) < 15000;

  switch (current) {
    case FlightPhase::PRELAUNCH: {
      // Establish baseline while quiet
      if (s.baro_health == SensorHealth::OK && amag > 5.f && amag < 15.f && gyro_mag < 30.f) {
        baseline_alt = isnan(baseline_alt) ? alt : (0.98f * baseline_alt + 0.02f * alt);
      }
      const bool accel_liftoff = amag >= cfg.liftoffAccelMagMs2
                                 || fabsf(axial) >= (9.81f + cfg.liftoffAxialDeltaMs2);
      if ((armed_or_countdown || near_ign) && accel_liftoff) {
        if (condition_since_ms == 0 || heldMs(now_ms, condition_since_ms) == 0)
          condition_since_ms = now_ms;
        if (heldMs(now_ms, condition_since_ms) >= cfg.liftoffMinMs) {
          launch_alt = alt;
          setPhase(FlightPhase::LIFTOFF, now_ms);
        }
      } else {
        condition_since_ms = now_ms;
      }
      break;
    }
    case FlightPhase::LIFTOFF:
    case FlightPhase::POWERED_ASCENT: {
      if (current == FlightPhase::LIFTOFF) setPhase(FlightPhase::POWERED_ASCENT, now_ms);
      // Burnout: accel falls toward ~1g after sustained boost, or max phase time
      if (amag < cfg.burnoutAccelMs2 && heldMs(now_ms, phase_since_ms) > cfg.burnoutMinMs) {
        setPhase(FlightPhase::COAST, now_ms);
      } else if (heldMs(now_ms, phase_since_ms) > 8000) {
        // safety: model motors burn out quickly
        setPhase(FlightPhase::COAST, now_ms);
      }
      break;
    }
    case FlightPhase::COAST: {
      // Apogee: vertical velocity near zero after altitude gain
      const float agl = isnan(baseline_alt) ? 0.f : (alt - baseline_alt);
      if (agl >= cfg.minFlightAltGainM && fabsf(vz) <= cfg.apogeeVertVelMs) {
        if (heldMs(now_ms, condition_since_ms) >= cfg.apogeeMinMs) {
          setPhase(FlightPhase::APOGEE, now_ms);
        }
      } else if (vz < cfg.descentVertVelMs && agl >= cfg.minFlightAltGainM * 0.5f) {
        setPhase(FlightPhase::DESCENT, now_ms);
      } else {
        condition_since_ms = now_ms;
      }
      break;
    }
    case FlightPhase::APOGEE: {
      if (vz < cfg.descentVertVelMs || heldMs(now_ms, phase_since_ms) > 500) {
        setPhase(FlightPhase::DESCENT, now_ms);
      }
      break;
    }
    case FlightPhase::DESCENT:
    case FlightPhase::IMPACT: {
      if (amag >= cfg.impactAccelMagMs2) {
        impact_flag = true;
        if (current != FlightPhase::IMPACT) setPhase(FlightPhase::IMPACT, now_ms);
      }
      // Multi-criteria landing confirmation — never single accel sample
      const float g = 9.81f;
      const bool quiet_accel = fabsf(amag - g) <= cfg.landedAccelBandMs2;
      const bool quiet_gyro = gyro_mag <= cfg.landedGyroMaxDps;
      const bool quiet_vz = fabsf(vz) <= cfg.landedVertVelMs;
      const bool quiet_alt = altStd() <= cfg.landedAltStdM;
      const bool had_flight = peak_agl >= cfg.minFlightAltGainM || impact_flag;

      if (had_flight && quiet_accel && quiet_gyro && quiet_vz && quiet_alt) {
        quiet_accum_ms += dt;
        if (quiet_accum_ms >= cfg.landedConfirmMs) {
          setPhase(FlightPhase::LANDED, now_ms);
        }
      } else {
        if (quiet_accum_ms > dt * 2) quiet_accum_ms -= dt * 2;
        else quiet_accum_ms = 0;
      }
      break;
    }
    case FlightPhase::LANDED:
    default:
      break;
  }

  return current != prev;
}

}  // namespace FlightDetect
