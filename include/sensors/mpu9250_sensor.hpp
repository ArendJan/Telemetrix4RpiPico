#pragma once
#include "sensor.hpp"

#include "drivers/MPU9250.hpp"

class MPU9250_Sensor : public Sensor {
public:
  MPU9250_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]);
  void readSensor();
  void resetSensor() {};

private:
  bool enabled = true;
  MPU9250 sensor;
  int i2c_port = 0;
};
