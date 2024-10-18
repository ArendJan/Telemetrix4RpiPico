#include "sensors/sensor.hpp"

#include "Telemetrix4RpiPico.hpp"
#include "message_types.hpp"

void Sensor::writeSensorData(std::vector<uint8_t> data) {
  std::vector<uint8_t> out = {
      0,                  // write len
      SENSOR_REPORT,      // write type
      (uint8_t)this->num, // write num
      this->type,         // write sensor type
  };
  out.insert(out.end(), data.begin(), data.end());
  out[0] = out.size() - 1; // dont count the packet length

  serial_write(out);
}