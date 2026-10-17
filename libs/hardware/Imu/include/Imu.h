#pragma once

// FreeInk inertial measurement unit (LSM6DS3TR-C or QMI8658, 6-axis accel +
// gyro; SC7A20H, 3-axis accelerometer only).
//
// Reads acceleration (g) and angular rate (deg/s) from the I2C IMU described by
// BoardConfig::ACTIVE.sensors (imuAddr / sensor bus). Dependency-free Wire
// access, mirroring BatteryMonitor. Boards without an IMU (FREEINK_CAP_IMU off,
// or imuAddr == 0) link stub bodies and present() returns false.
//
// ImuType::Sc7a20h has NO gyroscope: read() fills ax/ay/az and leaves gx/gy/gz
// at 0. Read Pico keeps the 12.5 Hz low-power profile. Metalio E-Ink 4 uses
// 100 Hz high-resolution mode, probes 0x19 then 0x18, and read() returns false
// while asleep or until STATUS ZYXDA is set. See metalio-accelerometer.md.

#include <Arduino.h>

#include <cstdint>

namespace freeink {

class Imu {
 public:
  struct Sample {
    float ax, ay, az;  // acceleration, g (1 g ~= 9.81 m/s^2)
    float gx, gy, gz;  // angular rate, degrees/second
  };

  // Verifies WHO_AM_I and configures accel + gyro for the active board.
  // Returns false when the active board has no IMU or the part doesn't identify.
  bool begin();
  bool present() const { return begun_; }

  // Reads one accel + gyro sample. Returns false on I2C error.
  bool read(Sample& out);

  // Puts the sensors into hardware standby / power-down. Config registers are
  // retained, so wake() restores sampling without a full begin(). Returns
  // false when the IMU is absent or on I2C error.
  bool sleep();

  // Restarts sampling after sleep(). Returns false when absent or on I2C
  // error; allow for a settling transient before trusting samples.
  bool wake();

  // False for an accelerometer-only part, and before begin() succeeds.
  bool hasGyroscope() const;

 private:
  bool begun_ = false;
  // Cleared by sleep() and failed begin()/wake(). read() returns false while
  // this is clear, including when a powered-down SC7A20H still holds output data.
  bool sampling_ = false;
  // The QMI8658 can legally appear at 0x6A or 0x6B depending on its SA0
  // strap. Keep the address found by begin() instead of repeatedly using the
  // board profile's preferred address.
  uint8_t addr_ = 0;
};

}  // namespace freeink

using Imu = freeink::Imu;
