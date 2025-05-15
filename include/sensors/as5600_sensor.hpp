#pragma once
#include "sensor.hpp"
#include "drivers/AS5600.hpp"
#include "drivers/TCA9548.hpp"
#include "i2c_helpers.hpp"
#include <vector>
// #include <pair>
class AS5600;
class AS5600_Sensor : public Sensor {
public:
AS5600_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]);
  void readSensor();
  void resetSensor() {};

private:
  void init_sequence();
  int i2c_port = 0;
  int i2c_addr = 83;
  uint8_t i2c_mux_ports = 0;
    TCA9548 *mux;
  std::vector<std::pair<int, AS5600>> as_sensors;
};