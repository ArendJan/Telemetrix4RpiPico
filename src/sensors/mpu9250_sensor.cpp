#include "sensors/mpu9250_sensor.hpp"

MPU9250_Sensor::MPU9250_Sensor(uint8_t config_data[SENSORS_MAX_SETTINGS_A]) {
  this->sensor.bus = config_data[0];
  auto settings = MPU9250Setting();

  settings.accel_fs_sel = ACCEL_FS_SEL::A2G;
  settings.gyro_fs_sel = GYRO_FS_SEL::G500DPS;

  auto i = this->sensor.setup(config_data[1], settings, config_data[0]);
  if (!i) {
    this->enabled = false;
  }
}

void MPU9250_Sensor::readSensor() {
  if (!this->enabled) {
    return;
  }
  if (this->sensor.update()) {
    std::vector<float> float_data;
    for (int i = 0; i < 3; i++) {
      float_data.push_back(this->sensor.getAcc(i));
    }
    for (int i = 0; i < 3; i++) {
      float_data.push_back(this->sensor.getGyro(i));
    }
    for (int i = 0; i < 3; i++) {
      float_data.push_back(this->sensor.getMag(i));
    }

    // Read out quaternion estimation
    float_data.push_back(this->sensor.getQuaternionX());
    float_data.push_back(this->sensor.getQuaternionY());
    float_data.push_back(this->sensor.getQuaternionZ());
    float_data.push_back(this->sensor.getQuaternionW());

    // This is fine, bytes are in weird order (but tmx-cpp knows)
    const unsigned char *bytes =
        reinterpret_cast<const uint8_t *>(&float_data[0]);
    static_assert(sizeof(float) == 4);
    std::vector<uint8_t> data(bytes, bytes + sizeof(float) * float_data.size());
    this->writeSensorData(data);
  }
}