#pragma once
#include "sensor.hpp"
#include "drivers/ICM20948.hpp"
class ICM20948_Sensor : public Sensor {
public:
  ICM20948_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]);
  void readSensor();
  void resetSensor(){};

private:
  int i2c_port = 0;
  int i2c_addr = 83;
  ICM20948 icm;
};