#pragma once
/**
 * Hardware pin map for NL-1 Pioneer onboard rocket computer.
 *
 * Sources of truth (do not invent pins):
 *   - electronics/firmware/connections.md
 *   - electronics/onboard-systems/onboard-systems.kicad_sch
 *   - Seeed Studio XIAO ESP32-S3 Sense official pinout
 *
 * External sensors on shared I2C bus:
 *   BMP580  (U2) — pressure + temperature
 *   LSM6DSO32 (U3) — accel + gyro (+ IMU die temperature)
 *
 * Onboard Sense expansion (camera + microSD) uses fixed GPIOs documented by Seeed.
 */

#include <stdint.h>

// ── Board identity ──────────────────────────────────────────────────────────
#define BOARD_MODEL_NAME        "Seeed XIAO ESP32-S3 Sense"
#define CAMERA_MODEL_NAME       "OV2640/OV3660 (XIAO Sense expansion)"
#define HAS_PSRAM_EXPECTED      1
#define PSRAM_SIZE_BYTES_EXPECT (8 * 1024 * 1024)

// ── External I2C bus (connections.md) ───────────────────────────────────────
// ESP32 D4 = GPIO5 = SDA
// ESP32 D5 = GPIO6 = SCL
static constexpr int PIN_I2C_SDA = 5;
static constexpr int PIN_I2C_SCL = 6;
static constexpr uint32_t I2C_FREQ_HZ = 400000;

// Default 7-bit addresses for Adafruit STEMMA QT breakouts (I2C mode, SDO unused)
// Adafruit_BMP5xx: DEFAULT 0x46, ALTERNATIVE 0x47
static constexpr uint8_t ADDR_BMP580     = 0x46;
static constexpr uint8_t ADDR_BMP580_ALT = 0x47;
static constexpr uint8_t ADDR_LSM6DSO32  = 0x6A;  // Adafruit default
static constexpr uint8_t ADDR_LSM6DSO32_ALT = 0x6B;

// ── Onboard microSD (XIAO ESP32-S3 Sense) ───────────────────────────────────
// Seeed pin multiplexing docs: SD occupies D8/D9/D10 + CS on GPIO21
static constexpr int PIN_SD_CS   = 21;
static constexpr int PIN_SD_MOSI = 9;   // D10
static constexpr int PIN_SD_MISO = 8;   // D9
static constexpr int PIN_SD_SCK  = 7;   // D8

// ── Onboard camera (XIAO ESP32-S3 Sense expansion) ──────────────────────────
// Confirmed against Seeed camera pin table (GPIO occupancy of Sense board).
static constexpr int CAM_PIN_PWDN  = -1;
static constexpr int CAM_PIN_RESET = -1;
static constexpr int CAM_PIN_XCLK  = 10;
static constexpr int CAM_PIN_SIOD  = 40;
static constexpr int CAM_PIN_SIOC  = 39;
static constexpr int CAM_PIN_D7    = 48;  // Y9
static constexpr int CAM_PIN_D6    = 11;  // Y8
static constexpr int CAM_PIN_D5    = 12;  // Y7
static constexpr int CAM_PIN_D4    = 14;  // Y6
static constexpr int CAM_PIN_D3    = 16;  // Y5
static constexpr int CAM_PIN_D2    = 18;  // Y4
static constexpr int CAM_PIN_D1    = 17;  // Y3
static constexpr int CAM_PIN_D0    = 15;  // Y2
static constexpr int CAM_PIN_VSYNC = 38;
static constexpr int CAM_PIN_HREF  = 47;
static constexpr int CAM_PIN_PCLK  = 13;

// ── Power ───────────────────────────────────────────────────────────────────
// Battery connects through SW1 to BAT+/BAT- (connections.md).
// No external battery ADC divider is present on the schematic — voltage
// measurement is therefore not available as a dedicated sensing path.
#define BATTERY_VOLTAGE_MEASURABLE 0

// User LED on XIAO ESP32-S3 (amber, active-low: LOW = on, HIGH = off).
// On Sense this pin is shared with microSD CS (PIN_SD_CS). Only drive the LED
// when SD is idle, and leave the pin HIGH afterward (LED off / CS deselect).
static constexpr int PIN_USER_LED = 21;
static constexpr bool USER_LED_ACTIVE_LOW = true;
#define USER_LED_AVAILABLE 1
