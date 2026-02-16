#pragma once
#include "sensor.hpp"

#include "drivers/HX711.hpp"

class HX711_Sensor : public Sensor {
public:
  HX711_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]);
  void readSensor();
  void resetSensor() {};

private:
  HX711 sensor = HX711();
};