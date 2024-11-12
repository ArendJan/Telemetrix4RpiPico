#pragma once
#include "sensor.hpp"

class VEML6040_Sensor : public Sensor {
public:
  VEML6040_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]);
  void readSensor();
  void resetSensor() {};

private:
  void init_sequence();
  int i2c_port = 0;
  int i2c_addr = 0x10;
};