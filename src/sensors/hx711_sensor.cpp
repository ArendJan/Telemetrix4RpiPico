#include "sensors/hx711_sensor.hpp"

#include "serialization.hpp"

HX711_Sensor::HX711_Sensor(uint8_t sensor_data[SENSORS_MAX_SETTINGS_A]) {
  auto dout = sensor_data[0];
  auto sck = sensor_data[1];
  this->sensor.begin(dout, sck);
}

void HX711_Sensor::readSensor() {
  if (this->sensor.is_ready()) {
    auto reading = this->sensor.read();
    static_assert(sizeof(reading) == 4);
    std::vector<uint8_t> data = encode_i32(reading);
    this->writeSensorData(data);
  }
}