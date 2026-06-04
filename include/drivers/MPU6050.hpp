#pragma once

#include <hardware/i2c.h>
#include <stdint.h>
// driver for MPU6050, 6-axis IMU
#include <stdint.h>
class MPU6050 {
public:
  MPU6050(i2c_inst_t *i2c_port, uint8_t addr);
  void read(int16_t accel[3], int16_t gyro[3], int16_t *temp);
  void reset();
  bool ok = true; // tracks whether the device is responding correctly, if not
                  // we skip reads to avoid blocking

private:
  i2c_inst_t *i2c_port;
  uint8_t addr;
};