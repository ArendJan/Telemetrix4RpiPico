#include "sensors/hmc5883l_sensor.hpp"

HMC5883L_Sensor::HMC5883L_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]) {
  int i2c_port = settings[0];
  // this->i2c_addr = settings[1];
  // this->init_sequence();
  this->sensor = new HMC5883L();
  this->sensor->i2c_port = i2c_port;
  auto ok = this->sensor->begin();
  if (!ok) {
    this->stop = true;
  }
  this->sensor->setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  this->sensor->setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  this->sensor->setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  this->sensor->setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  this->sensor->setOffset(0, 0, 0);
}

void HMC5883L_Sensor::readSensor() {
  if (this->stop) {
    return;
  }
  std::vector<uint8_t> data;
  auto v = this->sensor->readNormalize();
  static_assert(sizeof(v) == (4 * 3)); // 3 floats

  for (auto i = 0; i < 3; i++) { // 3 floats, xyz
    auto f = v.data[i];
    const unsigned char *bytes = reinterpret_cast<const uint8_t *>(&f);
    data.insert(data.end(), bytes, bytes + sizeof(f));
    static_assert(sizeof(f) == 4);
  }
  this->writeSensorData(data);
}
