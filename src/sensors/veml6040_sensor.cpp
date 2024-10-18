#include "sensors/veml6040_sensor.hpp"
#include "i2c_helpers.hpp"

VEML6040_Sensor::VEML6040_Sensor(uint8_t settings[SENSORS_MAX_SETTINGS_A]) {
  this->i2c_port = settings[0];
  this->init_sequence();
}

void VEML6040_Sensor::init_sequence() {
  bool ok = write_i2c(this->i2c_port, this->i2c_addr,
                      {
                          0, // register 0
                          // 0b000 << 4 |   // timing 40ms
                          //     0b0 << 2 | // no trigger
                          //     0b0 << 1 | // auto mode
                          //     0b0,       // enable sensor
                          // 0b0            // reserved H byte
                          0x21 // 0x21 = 0b0010'0001, 100 time, stop bit
                      });
  sleep_ms(10);
  ok &= write_i2c(this->i2c_port, this->i2c_addr,
                  {
                      0, // register 0
                      // 0b000 << 4 |   // timing 40ms
                      //     0b0 << 2 | // no trigger
                      //     0b0 << 1 | // auto mode
                      //     0b0,       // enable sensor
                      // 0b0            // reserved H byte
                      0x20 // 0x20 = 0b0010'0000, 100 time, no stop bit
                  });
  if (!ok) {
    this->stop = true;
  }
}

void VEML6040_Sensor::readSensor() {
  if (this->stop) {
    return;
  }
  std::vector<uint8_t> data;
  data.reserve(8);
  std::vector<uint8_t> single_color_data(2);
  bool ok = true;
  for (uint8_t reg = 0x08; reg <= 0x0B;
       reg++) { // read the 4 registers and add the data to the full data vector
    ok &= read_i2c(this->i2c_port, this->i2c_addr, {reg}, 2, single_color_data);
    data.push_back(single_color_data[0]);
    data.push_back(single_color_data[1]);
  }

  this->writeSensorData(data);
  if (!ok) {
    this->stop = true;
  }
}