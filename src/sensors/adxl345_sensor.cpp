#include "i2c_helpers.hpp"

#include "sensors/adxl345_sensor.hpp"

ADXL345_Sensor::ADXL345_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]) {
  // assume i2c is done
  this->i2c_port = settings[0];
  this->init_sequence();
}

void ADXL345_Sensor::init_sequence() {
  // write 83, 42, 0
  // Disable TAP detection
  bool ok = write_i2c(this->i2c_port, this->i2c_addr, {0x2A, 0});

  // write 83, 45, 8
  // Set POWER_CTL to enable measuring
  ok &= write_i2c(this->i2c_port, this->i2c_addr, {0x2D, 0b0000'1000});

  // write 83, 49, 8
  // Set DATA_FORMAT to FULLRANGE
  // TODO: Maybe 2G range better like MPU? (Probably fine?)
  ok &= write_i2c(this->i2c_port, this->i2c_addr, {0x31, 0b0000'1000});
  if (!ok) {
    this->stop = true;
  }
}

void ADXL345_Sensor::readSensor() {
  if (this->stop) {
    return;
  }

  // Read the data from Data X0 to data Z1
  // read 83, 50, 6bytes
  std::vector<uint8_t> out(6);
  this->stop = !read_i2c(this->i2c_port, this->i2c_addr, {0x32}, 6, out);

  // Swap to make BigEndian
  for (int idx = 0; idx < 3; idx++)
    std::swap(out[2 * idx], out[2 * idx + 1]);

  this->writeSensorData(out);
}
