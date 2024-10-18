//
// Created by afy on 3/3/21.
//
#pragma once

#include "module/module.hpp"
#include "sensors/sensor.hpp"
#include "Telemetrix4RpiPico.pio.h"
#include "message_types.hpp"
#include "uart.hpp"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "hardware/watchdog.h"
#include "i2c_helpers.hpp"
#include "math.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include <array>
#include <pico/sync.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
/************************** FORWARD REFERENCES ***********************
We define all functions here as extern to provide allow
forward referencing.
**********************************************************************/

extern void serial_loopback();

extern void set_pin_mode();

extern void digital_write();

extern void pwm_write();

extern void modify_reporting();

extern void get_firmware_version();

extern void get_pico_unique_id();

extern void servo_attach();

extern void servo_write();

extern void servo_detach();

extern void i2c_begin();

extern void i2c_read();

extern void i2c_write();

extern void sonar_new();

extern void led_debug(int blinks, uint delay);

extern void send_debug_info(uint id, uint value);

extern void dht_new();

extern void stop_all_reports();

extern void enable_all_reports();

extern void reset_data();

extern void reset_board();

extern void scan_digital_inputs();

extern void scan_analog_inputs();

extern void init_neo_pixels();

extern void show_neo_pixels();

extern void set_neo_pixel();

extern void clear_all_neo_pixels();

extern void fill_neo_pixels();

extern void read_sonar(uint);

extern void read_dht(uint);

extern void init_spi();

extern void read_blocking_spi();

extern void write_blocking_spi();

extern void spi_cs_control();

extern void set_format_spi();

extern void set_scan_delay();

void encoder_new();

void check_uart_loopback();
extern volatile bool uart_enabled;
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Resets all hardware devices
void reset_hardware();

// a descriptor for digital pins
typedef struct {
  uint pin_number;
  uint pin_mode;
  uint reporting_enabled; // If true, then send reports if an input pin
  int last_value;         // Last value read for input mode
} pin_descriptor;

// a descriptor for analog pins
typedef struct analog_pin_descriptor {
  uint reporting_enabled; // If true, then send reports if an input pin
  int last_value;         // Last value read for input mode
  int differential;       // difference between current and last value needed
} analog_pin_descriptor;

// This structure describes an HC-SR04 type device
typedef struct hc_sr04_descriptor {
  uint trig_pin; // trigger pin
  uint echo_pin; // echo pin
  uint32_t start_time;
  uint32_t last_time_diff;
  int last_dist;
} hc_sr04_descriptor;

// this structure holds an index into the sonars array
// and the sonars array
typedef struct sonar_data {
  int next_sonar_index;
  repeating_timer_t trigger_timer;
  uint32_t trigger_mask;
  hc_sr04_descriptor sonars[MAX_SONARS];
  mutex_t mutex;
} sonar_data;

// this structure describes a DHT type device
typedef struct dht_descriptor {
  uint data_pin; // data pin
  absolute_time_t previous_time;
  /* for possible future use
  int last_val_whole; // value on right side of decimal
  int last_val_frac; // value on left side of decimal
  */
} dht_descriptor;

// this structure holds an index into the dht array
// and the dhts array
typedef struct dht_data {
  int next_dht_index;
  dht_descriptor dhts[MAX_DHTS];
} dht_data;

// encoder type
typedef struct {
  ENCODER_TYPES type;
  int A;
  int B;
  int8_t step;
  int last_state;
} encoder_t;

typedef struct {
  int next_encoder_index;
  repeating_timer_t trigger_timer;
  encoder_t encoders[MAX_ENCODERS];
  mutex_t mutex;
} encoder_data;

extern encoder_data encoders;

typedef struct {
  // a pointer to the command processing function
  void (*command_func)(void);
} command_descriptor;

extern bool watchdog_enable_shutdown; // if false, then don't do anything with
                                      // the watchdog and just wait for shutdown
void ping();

void sensor_new();
// void addSensor(uint8_t data[], size_t len);
void readSensors();
extern std::vector<Sensor *> sensors;
void reportBytes(std::vector<uint8_t>);

// class Shutdown_Relay : public Module {
// public:
//   Shutdown_Relay(std::vector<uint8_t> &data);
//   void readModule();
//   void writeModule(std::vector<uint8_t> &data);
//   void resetModule(){};

// private:
//   int pin = 0;
//   bool enabled = false;
//   bool enable_on = true;
//   int wait_time = 10;
//   decltype(time_us_32()) start_time = 0;
// };

extern std::vector<Module *> modules;
void module_new();
void module_data();

const auto WATCHDOG_TIME = 5'000; // 5s timeout

extern volatile bool uart_enabled;

bool timeout_safe();

extern void led_debug(int blinks, uint delay);

void get_id();
void set_id();
