#pragma once
#include "sensor.hpp"

class ADXL345_Sensor : public Sensor {
public:
  ADXL345_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]);
  void readSensor();
  void resetSensor(){};

private:
  void init_sequence();
  int i2c_port = 0;
  int i2c_addr = 83;
};