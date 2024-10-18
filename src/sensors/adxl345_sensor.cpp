#include "i2c_helpers.hpp"

#include "sensors/adxl345_sensor.hpp"

ADXL345_Sensor::ADXL345_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]) {
  // assume i2c is done
  this->i2c_port = settings[0];
  this->init_sequence();
}

void ADXL345_Sensor::init_sequence() {
  // write 83, 43, 0
  bool ok = write_i2c(this->i2c_port, this->i2c_addr, {43, 0});
  // write 83, 45, 8
  ok &= write_i2c(this->i2c_port, this->i2c_addr, {45, 8});
  // write 83, 49, 8
  ok &= write_i2c(this->i2c_port, this->i2c_addr, {49, 8});
  if (!ok) {
    this->stop = true;
  }
}

void ADXL345_Sensor::readSensor() {
  if (this->stop) {
    return;
  }
  // read 83, 50, 6bytes
  std::vector<uint8_t> out(6);
  this->stop = !read_i2c(this->i2c_port, this->i2c_addr, {50}, 6, out);

  this->writeSensorData(out);
}