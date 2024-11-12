#include "i2c_helpers.hpp"

#include "sensors/ICM20948_sensor.hpp"

ICM20948_Sensor::ICM20948_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]) {
  // assume i2c is done
  this->i2c_port = settings[0];
  // this->init_sequence();
  this->stop = !this->icm.init(this->i2c_port, this->i2c_addr);
  if(stop) {
    return; // TODO: report failing init
  }
}

void ICM20948_Sensor::readSensor() {
  if (this->stop) {
    return;
  }
  uint16_t ang[3];
  uint16_t acc[3];
  uint16_t mag[3];
  uint16_t pos[3];
  this->icm.read_data(ang, acc, mag, pos);
  std::vector<uint8_t> out;
  for(auto arr : {ang, acc, mag, pos}) {
    for(int i = 0; i < 3; i++) {
      out.push_back((arr[i] >> 8) & 0xFF);
      out.push_back(arr[i] & 0xFF);
    }
  }
  for(int i = 0; i < 3; i++) {
    out.push_back((ang[i] >> 8) & 0xFF);
    out.push_back(ang[i] & 0xFF);
  }
  // Swap to make BigEndian
  // for (int idx = 0; idx < 3; idx++)
  //   std::swap(out[2 * idx], out[2 * idx + 1]);

  this->writeSensorData(out);
}
