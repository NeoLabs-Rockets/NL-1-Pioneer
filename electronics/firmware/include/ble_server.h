#pragma once
#include "types.h"
#include <stdint.h>

namespace BleServer {

void begin();
void end();

BluetoothState state();
bool connected();
uint32_t lastRxMs();

// Publish status JSON notify (compact, launch-controller style)
void publishStatus(const SystemSnapshot& snap, const SensorSnapshot& sensors);

// Publish compact binary or JSON telemetry notify
void publishTelemetry(const SensorSnapshot& sensors, const SystemSnapshot& snap);

// Drain and process inbound command queue (call from mission task)
void serviceCommands();

// Reduce telemetry rate under load
void setTelemetryHz(uint32_t hz);
uint32_t telemetryHz();

}  // namespace BleServer
