#pragma once
#include "sensor.hpp"

#include "drivers/VL53L0.hpp"

class VL53L0X_Sensor : public Sensor {
public:
  VL53L0X_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]);
  void readSensor();
  void resetSensor(){};

private:
  VL53L0X sensor;
  int i2c_port = 0;
  int i2c_addr = 0x52;
};