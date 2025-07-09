#include <sensors/as5600_sensor.hpp>

// #include "drivers/AS5600.hpp"
#include "i2c_helpers.hpp"
void send_debug_info(uint id, uint value);

AS5600_Sensor::AS5600_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]) {

  int i2c_port = settings[0];
  this->i2c_mux_ports = settings[1];
  this->i2c_port = i2c_port;
  auto mux = this->i2c_mux_ports;
  auto num = 0;
  while (mux > 0) {
    if (mux & 1) {
      this->as_sensors.push_back(std::make_pair(num, AS5600(i2c_port)));
    }
    mux = mux >> 1;
    num++;
  }
  this->mux = new TCA9548(0x70, i2c_port);
  auto ok = this->mux->begin();
  for (auto &as_sensor : this->as_sensors) {
    this->mux->selectChannel(as_sensor.first);
    ok |= as_sensor.second.begin();
  }
  this->mux->disableAllChannels();
}

void AS5600_Sensor::readSensor() {
  if (this->stop) {
    return;
  }
  std::vector<uint8_t> data;
  for (auto &as_sensor : this->as_sensors) {
    this->mux->selectChannel(as_sensor.first);
    auto angle = as_sensor.second.readAngle();
    data.push_back((uint8_t)(angle >> 8));
    data.push_back((uint8_t)(angle & 0xFF));
  }
  this->writeSensorData(data);
  this->mux->disableAllChannels();
}