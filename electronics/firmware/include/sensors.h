#pragma once
#include "types.h"

namespace Sensors {

bool begin();  // probe I2C devices
void end();

// Non-blocking sample update; call from sensor task at SENSOR_HZ_IMU.
bool sample(SensorSnapshot& out);

SensorHealth imuHealth();
SensorHealth baroHealth();
bool imuPresent();
bool baroPresent();

// Ground-test overrides
void setSimulation(bool enabled);
void injectSim(const SensorSnapshot& s);

}  // namespace Sensors
