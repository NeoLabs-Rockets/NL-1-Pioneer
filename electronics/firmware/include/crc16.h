#pragma once
#include <stdint.h>
#include <stddef.h>

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) — compact integrity check for
// binary telemetry records and BLE frames.
uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t seed = 0xFFFF);
