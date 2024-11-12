#pragma once
#include <stdint.h>
#include <vector>

/*****************************************************************************/
/****SENSORS*/
/*****************************************************************************/

enum SENSOR_TYPES : uint8_t { // Max 255 sensors, but will always fit in a
                              // single byte!
  GPS = 0,
  LOAD_CELL = 1,
  MPU_9250 = 2,
  TOF_VL53 = 3,
  VEML6040 = 4, // Color sensor
  ADXL345 = 5,  // 3 axis accel
  INA226a = 6,
  HMC5883l = 7,
  ICM20948_sensor = 8,
  MAX_SENSORS
};

class Sensor {
public:
  virtual void readSensor() = 0;
  virtual void resetSensor() = 0;
  bool stop = false;
  void writeSensorData(std::vector<uint8_t> data);
  int num;
  SENSOR_TYPES type = SENSOR_TYPES::MAX_SENSORS;
};

const int SENSORS_MAX_SETTINGS_A = 6;