#pragma once
#include "types.h"
#include "config.h"

namespace FlightDetect {

void begin(const FlightDetectConfig& cfg = kFlightCfg);
void reset();

// Feed a new sensor sample; returns true if phase changed.
bool update(const SensorSnapshot& s, int64_t expected_ignition_mono_ms, bool armed_or_countdown);

FlightPhase phase();
float peakAltitudeAglM();
float baselineAltitudeM();
bool sawImpact();
uint32_t quietMs();

// Ground-test injection of synthetic samples.
void injectPhase(FlightPhase p);

}  // namespace FlightDetect
