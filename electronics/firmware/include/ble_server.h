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

// Notify binary file chunk: header + payload
// Layout: [u32 offset LE][u32 total LE][u16 len LE][data...]
bool notifyFileChunk(uint32_t offset, uint32_t total, const uint8_t* data, uint16_t len);

// Notify text/JSON on event characteristic (command results, file list, etc.)
void notifyEventJson(const char* json);

}  // namespace BleServer
