
#include "drivers/MPU6050.hpp"
#include "i2c_helpers.hpp"
#include <stdio.h>
MPU6050::MPU6050(i2c_inst_t *i2c_port, uint8_t addr)
    : i2c_port(i2c_port), addr(addr) {
  reset();
}

bool send_command_mpu(i2c_inst_t *i2c_port, uint8_t addr, uint8_t reg,
                      uint8_t data) {
  uint8_t buf[] = {reg, data};
  auto out = i2c_write_blocking(i2c_port, addr, buf, 2, false);
  return out == 2;
}

// Code taken from
// https://github.com/raspberrypi/pico-examples/blob/master/i2c/mpu6050_i2c/mpu6050_i2c.c
void MPU6050::reset() {
  // Two byte reset. First byte register, second byte data
  // There are a load more options to set up the device in different ways that
  // could be added here

  uint8_t buf[] = {
      0x6B,
      0x80}; // 0x6B is the power management register, 0x80 is the reset command
  auto out = i2c_write_blocking_until(this->i2c_port, addr, buf, 2, false,
                                      make_timeout_time_ms(10));
  if (out != 2) {
    // printf("MPU6050 reset failed with return value %d\n", out);
    this->ok = false;
    return;
  }
  sleep_ms(100); // Allow device to reset and stabilize

  // Clear sleep mode (0x6B register, 0x00 value)
  buf[1] = 0x00; // Clear sleep mode by writing 0x00 to the 0x6B register
  out = i2c_write_blocking_until(this->i2c_port, addr, buf, 2, false,
                                 make_timeout_time_ms(10));
  if (out != 2) {
    // printf("MPU6050 wake failed with return value %d\n", out);
    this->ok = false;
    return;
  }

  ok &= send_command_mpu(this->i2c_port, this->addr, 0x1B,
                         0x00); // Set gyro full scale to ±250 deg/s
  ok &= send_command_mpu(this->i2c_port, this->addr, 0x1C,
                         0x00); // Set accel full scale to ±2g
  ok &= send_command_mpu(this->i2c_port, this->addr, 0x19,
                         0x07); // Set sample rate to 1kHz/(1+7) = 125Hz

  sleep_ms(10); // Allow stabilization after waking up
}

void MPU6050::read(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
  // For this particular device, we send the device the register we want to read
  // first, then subsequently read from the device. The register is auto
  // incrementing so we don't need to keep sending the register we want, just
  // the first.
  if (!ok) {
    return;
  }
  uint8_t buffer[6];

  // Start reading acceleration registers from register 0x3B for 6 bytes
  uint8_t val = 0x3B;
  auto out = i2c_write_blocking_until(
      this->i2c_port, addr, &val, 1, true,
      make_timeout_time_ms(10)); // true to keep master control of bus
  if (out != 1) {
    this->ok = false;
    return;
  }
  out = i2c_read_blocking_until(this->i2c_port, addr, buffer, 6, false,
                                make_timeout_time_ms(10));
  if (out != 6) {
    this->ok = false;
    return;
  }
  for (int i = 0; i < 3; i++) {
    accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
  }

  // Now gyro data from reg 0x43 for 6 bytes
  // The register is auto incrementing on each read
  val = 0x43;
  out = i2c_write_blocking_until(this->i2c_port, addr, &val, 1, true,
                                 make_timeout_time_ms(10));
  if (out != 1) {
    this->ok = false;
    return;
  }
  out = i2c_read_blocking_until(
      this->i2c_port, addr, buffer, 6, false,
      make_timeout_time_ms(10)); // False - finished with bus
  if (out != 6) {
    this->ok = false;
    return;
  }

  for (int i = 0; i < 3; i++) {
    gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    ;
  }

  // Now temperature from reg 0x41 for 2 bytes
  // The register is auto incrementing on each read
  val = 0x41;
  out = i2c_write_blocking_until(this->i2c_port, addr, &val, 1, true,
                                 make_timeout_time_ms(10));
  if (out != 1) {
    this->ok = false;
    return;
  }
  out = i2c_read_blocking_until(
      this->i2c_port, addr, buffer, 2, false,
      make_timeout_time_ms(10)); // False - finished with bus
  if (out != 2) {
    this->ok = false;
    return;
  }

  *temp = buffer[0] << 8 | buffer[1];
}