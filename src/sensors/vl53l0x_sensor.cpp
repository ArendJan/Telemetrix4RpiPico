#include "sensors/vl53l0x_sensor.hpp"

VL53L0X_Sensor::VL53L0X_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]) {
  this->sensor.setBus(settings[0]);
  bool ok = this->sensor.init();
  // this->sensor.setTimeout(1);
  if (!ok) {
    this->stop = true;
    return;
  }
  this->sensor.startContinuous();
}

void VL53L0X_Sensor::readSensor() {
  auto dist = this->sensor.readRangeContinuousMillimeters();
  std::vector<uint8_t> data = {(uint8_t)(dist >> 8), (uint8_t)(dist & 0xFF)};
  this->writeSensorData(data);
}