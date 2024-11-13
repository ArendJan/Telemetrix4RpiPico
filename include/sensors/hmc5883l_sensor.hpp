#pragma once
#include "sensor.hpp"

#include "drivers/HMC5883L.hpp"

class HMC5883L_Sensor : public Sensor {
public:
  HMC5883L_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]);
  void readSensor();
  void resetSensor() {};

private:
  HMC5883L *sensor = nullptr;
};
