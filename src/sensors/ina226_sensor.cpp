#include "sensors/ina226_sensor.hpp"

#include "pico/time.h"

#include "Telemetrix4RpiPico.hpp"

INA226_Sensor::INA226_Sensor(uint8_t sensor_data[SENSORS_MAX_SETTINGS_A]) {
  auto port = sensor_data[0];
  auto addr = sensor_data[1];
  if (addr < 0b1000'000 || addr > 0b100'1111) {
    addr = 0b100'0000;
  }
  this->sensor = new INA226_WE(addr, port);
  if (!this->sensor->init()) {
    this->stop = true;
    return;
  }
  this->sensor->setResistorRange(0.010, 8.3);
  // this->sensor.av
  this->sensor->setAverage(INA226_AVERAGES::AVERAGE_256);
  this->sensor->setMeasureMode(INA226_MEASURE_MODE::CONTINUOUS);
}

void INA226_Sensor::readSensor() {
  static uint32_t last_scan = 0;
  if (this->stop) {
    send_debug_info(12, 14);
    return;
  }
  if (time_us_32() - last_scan <=
      200'000) // update every 200ms, sensor has new data every 1.1ms*256
  {
    return;
  }
  last_scan = time_us_32();
  std::vector<float> float_data;
  float f = this->sensor->getBusVoltage_V();
  float_data.push_back(f);
  float_data.push_back(this->sensor->getCurrent_A());
  const unsigned char *bytes =
      reinterpret_cast<const uint8_t *>(&float_data[0]);
  static_assert(sizeof(float) == 4);
  std::vector<uint8_t> data(bytes, bytes + sizeof(float) * float_data.size());
  this->writeSensorData(data);
}
