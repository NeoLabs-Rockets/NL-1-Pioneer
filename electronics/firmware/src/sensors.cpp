#include "sensors.h"
#include "board_pins.h"
#include "time_manager.h"
#include "event_log.h"
#include "config.h"

#ifndef UNIT_TEST
#include <Wire.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_BMP5xx.h>
#include <esp_system.h>
#endif

#include <math.h>
#include <string.h>

namespace Sensors {
namespace {

#ifndef UNIT_TEST
Adafruit_LSM6DSO32 lsm;
Adafruit_BMP5xx bmp;
#endif

bool imu_ok = false;
bool baro_ok = false;
SensorHealth imu_h = SensorHealth::MISSING;
SensorHealth baro_h = SensorHealth::MISSING;

float sea_level_pa = 101325.f;
float last_alt = NAN;
uint64_t last_alt_us = 0;
float vert_vel = 0;
float filt_ax = 0, filt_ay = 0, filt_az = 0;

bool sim_enabled = false;
SensorSnapshot sim_sample{};

float pressureToAltitude(float pa) {
  // International Standard Atmosphere approximation
  return 44330.f * (1.f - powf(pa / sea_level_pa, 0.1903f));
}

#ifndef UNIT_TEST
float readMcuTempC() {
  // ESP32-S3 internal temperature sensor (°C).
  return temperatureRead();
}
#endif

}  // namespace

#ifndef UNIT_TEST
// Scan 7-bit addresses and print responders for bring-up / wiring debug.
static int i2cScanAndLog(const char* tag, int sda, int scl, uint32_t hz) {
  Wire.end();
  // Weak internal pull-ups help if external STEMMA pull-ups are missing.
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  delay(2);
  int sda_lvl = digitalRead(sda);
  int scl_lvl = digitalRead(scl);
  Serial.printf("[NL1] I2C %s SDA=GPIO%d SCL=GPIO%d @ %lu Hz (idle SDA=%d SCL=%d, expect 1/1)\n",
                tag, sda, scl, static_cast<unsigned long>(hz), sda_lvl, scl_lvl);

  Wire.begin(sda, scl);
  Wire.setClock(hz);
  delay(20);

  int found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      found++;
      const char* who = "";
      if (addr == ADDR_BMP580 || addr == ADDR_BMP580_ALT) who = " (BMP580?)";
      else if (addr == ADDR_LSM6DSO32 || addr == ADDR_LSM6DSO32_ALT) who = " (LSM6DSO32?)";
      Serial.printf("[NL1]   I2C device at 0x%02X%s\n", addr, who);
    }
  }
  if (found == 0) {
    Serial.println("[NL1]   I2C: no devices responded");
  } else {
    Serial.printf("[NL1]   I2C: %d device(s) found\n", found);
  }
  return found;
}
#endif

bool begin() {
  imu_ok = false;
  baro_ok = false;
  imu_h = SensorHealth::MISSING;
  baro_h = SensorHealth::MISSING;
  last_alt = NAN;
  vert_vel = 0;
  sim_enabled = false;

#ifndef UNIT_TEST
  // PCB nets (onboard-systems.kicad_pcb):
  //   SDA = U1 D4/GPIO5 → U2.SDA + U3.SDA
  //   SCL = U1 D5/GPIO6 → U2.SCL + U3.SCL
  //   VIN = U1 3V3
  // Try 100 kHz first (more tolerant of capacitance / weak pull-ups), then 400 kHz.
  int found = i2cScanAndLog("primary", PIN_I2C_SDA, PIN_I2C_SCL, 100000);
  if (found == 0) {
    found = i2cScanAndLog("primary-400k", PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
  }
  if (found == 0) {
    // Diagnostic only: detect SDA/SCL swap on a hand-wired board.
    found = i2cScanAndLog("swapped", PIN_I2C_SCL, PIN_I2C_SDA, 100000);
    if (found > 0) {
      Serial.println("[NL1] WARNING: devices only visible with SDA/SCL swapped!");
    } else {
      // Restore primary pin map for library probes below.
      Wire.end();
      Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
      Wire.setClock(100000);
    }
  }

  delay(50);  // allow breakouts to come out of power-on reset

  // LSM6DSO32 — try both address straps
  if (lsm.begin_I2C(ADDR_LSM6DSO32, &Wire) || lsm.begin_I2C(ADDR_LSM6DSO32_ALT, &Wire)) {
    lsm.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
    lsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    lsm.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm.setGyroDataRate(LSM6DS_RATE_104_HZ);
    imu_ok = true;
    imu_h = SensorHealth::OK;
    Serial.println("[NL1] LSM6DSO32 OK");
  } else {
    EventLog::emit(EventType::SENSOR_FAULT, "LSM6DSO32_missing");
    imu_h = SensorHealth::MISSING;
    Serial.println("[NL1] LSM6DSO32 MISSING");
  }

  // BMP580 (Adafruit_BMP5xx: pressure field is hPa after performReading)
  if (bmp.begin(ADDR_BMP580, &Wire) || bmp.begin(ADDR_BMP580_ALT, &Wire)) {
    bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
    bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);
    baro_ok = true;
    baro_h = SensorHealth::OK;
    Serial.println("[NL1] BMP580 OK");
  } else {
    EventLog::emit(EventType::SENSOR_FAULT, "BMP580_missing");
    baro_h = SensorHealth::MISSING;
    Serial.println("[NL1] BMP580 MISSING");
  }
#else
  imu_ok = true;
  baro_ok = true;
  imu_h = SensorHealth::OK;
  baro_h = SensorHealth::OK;
#endif

  return imu_ok || baro_ok;
}

void end() {}

bool sample(SensorSnapshot& out) {
  if (sim_enabled) {
    out = sim_sample;
    out.ts = TimeManager::now();
    out.valid = true;
    return true;
  }

  memset(&out, 0, sizeof(out));
  out.ts = TimeManager::now();
  out.imu_health = imu_h;
  out.baro_health = baro_h;
  out.valid = false;

#ifndef UNIT_TEST
  // IMU
  if (imu_ok) {
    sensors_event_t accel, gyro, temp;
    if (lsm.getEvent(&accel, &gyro, &temp)) {
      // Light low-pass for noise without hiding liftoff
      const float a = 0.35f;
      filt_ax = a * accel.acceleration.x + (1 - a) * filt_ax;
      filt_ay = a * accel.acceleration.y + (1 - a) * filt_ay;
      filt_az = a * accel.acceleration.z + (1 - a) * filt_az;
      out.ax = filt_ax;
      out.ay = filt_ay;
      out.az = filt_az;
      out.gx = gyro.gyro.x * (180.f / static_cast<float>(M_PI));  // rad/s → deg/s if library uses rad
      // Adafruit LSM6DS gyro event is in rad/s
      out.gy = gyro.gyro.y * (180.f / static_cast<float>(M_PI));
      out.gz = gyro.gyro.z * (180.f / static_cast<float>(M_PI));
      out.accel_mag = sqrtf(out.ax * out.ax + out.ay * out.ay + out.az * out.az);

      out.temps[static_cast<int>(TempSensorId::LSM6DSO32)] = {
          TempSensorId::LSM6DSO32, temp.temperature, true, out.ts.mono_us};
      imu_h = SensorHealth::OK;
      out.valid = true;
    } else {
      imu_h = SensorHealth::FAILED;
      out.temps[static_cast<int>(TempSensorId::LSM6DSO32)] = {
          TempSensorId::LSM6DSO32, NAN, false, out.ts.mono_us};
      EventLog::emit(EventType::SENSOR_FAULT, "LSM6DSO32_read_fail");
    }
  } else {
    out.temps[static_cast<int>(TempSensorId::LSM6DSO32)] = {
        TempSensorId::LSM6DSO32, NAN, false, out.ts.mono_us};
  }

  // Barometer
  if (baro_ok) {
    if (bmp.performReading()) {
      // Library stores pressure in hPa — convert to Pa for telemetry.
      out.pressure_pa = bmp.pressure * 100.f;
      out.temps[static_cast<int>(TempSensorId::BMP580)] = {
          TempSensorId::BMP580, bmp.temperature, true, out.ts.mono_us};
      out.altitude_m = pressureToAltitude(out.pressure_pa);
      if (!isnan(last_alt) && last_alt_us > 0) {
        float dt = (out.ts.mono_us - last_alt_us) / 1e6f;
        if (dt > 0.001f && dt < 1.f) {
          float raw_v = (out.altitude_m - last_alt) / dt;
          vert_vel = 0.25f * raw_v + 0.75f * vert_vel;
        }
      }
      last_alt = out.altitude_m;
      last_alt_us = out.ts.mono_us;
      out.vert_vel_ms = vert_vel;
      baro_h = SensorHealth::OK;
      out.valid = true;
    } else {
      baro_h = SensorHealth::DEGRADED;
      out.temps[static_cast<int>(TempSensorId::BMP580)] = {
          TempSensorId::BMP580, NAN, false, out.ts.mono_us};
    }
  } else {
    out.temps[static_cast<int>(TempSensorId::BMP580)] = {
        TempSensorId::BMP580, NAN, false, out.ts.mono_us};
  }

  // MCU temperature
  float mcu_t = readMcuTempC();
  out.temps[static_cast<int>(TempSensorId::MCU_INTERNAL)] = {
      TempSensorId::MCU_INTERNAL, mcu_t, !isnan(mcu_t), out.ts.mono_us};

  out.imu_health = imu_h;
  out.baro_health = baro_h;
  if (!isnan(last_alt) && !isnan(out.altitude_m)) {
    // AGL relative to first good sample treated as pad — refined by FlightDetect baseline
    out.altitude_agl_m = out.altitude_m - last_alt;  // temporary; mission uses FlightDetect
  }
#else
  (void)out;
#endif
  return out.valid;
}

SensorHealth imuHealth() { return imu_h; }
SensorHealth baroHealth() { return baro_h; }
bool imuPresent() { return imu_ok; }
bool baroPresent() { return baro_ok; }

void setSimulation(bool enabled) { sim_enabled = enabled; }

void injectSim(const SensorSnapshot& s) {
  sim_enabled = true;
  sim_sample = s;
  sim_sample.valid = true;
}

}  // namespace Sensors
