#include "sensors/bno055_sensor.hpp"
#include "i2c_helpers.hpp"
#include "serialization.hpp"
BNO055_Sensor::BNO055_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]) {
  auto i2c_port = settings[0];
  i2c_inst_t *i = i2c_port == 1 ? i2c1 : i2c0;
  //   TODO: more info

  this->bno = new Adafruit_BNO055(0, BNO055_ADDRESS_A, i);
  if (!this->bno->begin()) {
    this->stop = true;
  }
  //   this->bno
}

// void BNO055_Sensor::init_sequence() {
//  if (!ok) {
//     this->stop = true;
//   }
// }
void send_debug_info(uint id, uint value);
auto const events = {
    // Adafruit_BNO055::VECTOR_EULER,
    Adafruit_BNO055::VECTOR_GYROSCOPE,
    // Adafruit_BNO055::VECTOR_LINEARACCEL,
    Adafruit_BNO055::VECTOR_MAGNETOMETER, Adafruit_BNO055::VECTOR_ACCELEROMETER,
    // Adafruit_BNO055::VECTOR_GRAVITY
};
void BNO055_Sensor::readSensor() {
  std::array<float, 3> data;
  std::vector<uint8_t> out;
  for (const auto &event : events) {
    bno->getEvent(data, event);
    for (int i = 0; i < 3; i++) {
      auto enc = encode_float(data[i]);
      out.insert(out.end(), enc.begin(), enc.end());
    }
  }
  auto quat = bno->getQuat();
  auto enc = encode_float(quat.x());
  out.insert(out.end(), enc.begin(), enc.end());
  enc = encode_float(quat.y());
  out.insert(out.end(), enc.begin(), enc.end());
  enc = encode_float(quat.z());
  out.insert(out.end(), enc.begin(), enc.end());
  enc = encode_float(quat.w());
  out.insert(out.end(), enc.begin(), enc.end());

  //   bno->getEvent(data, Adafruit_BNO055::VECTOR_EULER);
  //   bno->getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  //   bno->getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  //   bno->getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  //   bno->getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  //   bno->getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  //   printEvent(&orientationData);
  //   printEvent(&angVelocityData);
  //   printEvent(&linearAccelData);
  //   printEvent(&magnetometerData);
  //   printEvent(&accelerometerData);
  //   printEvent(&gravityData);

  int8_t boardTemp = bno->getTemp();
  out.push_back(boardTemp);
  this->writeSensorData(out);
  //   if (this->stop) {
  //     return;
  //   }
  //   std::vector<uint8_t> data;
  //   data.reserve(8);
  //   std::vector<uint8_t> single_color_data(2);
  //   bool ok = true;
  //   for (uint8_t reg = 0x08; reg <= 0x0B;
  //        reg++) { // read the 4 registers and add the data to the full data
  //        vector
  //     ok &= read_i2c(this->i2c_port, this->i2c_addr, {reg}, 2,
  //     single_color_data);
  //     // Flip the order to make it match the rest.
  //     data.push_back(single_color_data[1]);
  //     data.push_back(single_color_data[0]);
  //   }

  //   this->writeSensorData(data);
  //   if (!ok) {
  //     this->stop = true;
  //   }
}