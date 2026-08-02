#pragma once
/**
 * BLE OTA for the rocket computer — same protocol shape as Launch Controller:
 *   control (JSON begin/finish/abort/status)
 *   data    (binary: u32 offset LE + payload)
 *   status  (JSON notify)
 *
 * Blocked while recording or in-flight. Never triggers ignition.
 */

#include <stdint.h>
#include <stdbool.h>

namespace OtaService {

void begin();   // create characteristics on existing NimBLE service (call after service create)
void attachToService(void* nimbleService);  // NimBLEService*

void loop();    // timeouts + deferred reboot

bool active();
bool rebootPending();

// Called from BLE characteristic callbacks
void onControlWrite(const char* json, uint16_t connHandle);
void onDataWrite(const uint8_t* data, size_t len, uint16_t connHandle);

void publishStatus();

}  // namespace OtaService
