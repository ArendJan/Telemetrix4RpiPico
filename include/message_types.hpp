#pragma once
#include "pico/types.h"
/*********************************************************
 *                       COMMAND DEFINES
 ********************************************************/

// Commands -received by this sketch
// Add commands retaining the sequential numbering.
// The order of commands here must be maintained in the command_table below.
// #define SERIAL_LOOP_BACK 0
// #define SET_PIN_MODE 1
// #define DIGITAL_WRITE 2
// #define PWM_WRITE 3
// #define MODIFY_REPORTING \
//   4 // mode(all, analog, or digital), pin, enable or disable
// #define GET_FIRMWARE_VERSION 5
// #define GET_PICO_UNIQUE_ID 6
// #define SERVO_ATTACH 7 // unused
// #define SERVO_WRITE 8  // unused
// #define SERVO_DETACH 9 // unused
// #define I2C_BEGIN 10
// #define I2C_READ 11
// #define I2C_WRITE 12
// #define SONAR_NEW 13
// #define DHT_NEW 14
// #define STOP_ALL_REPORTS 15
// #define ENABLE_ALL_REPORTS 16
// #define RESET_DATA 17
// #define RESET_BOARD 18
// #define INITIALIZE_NEO_PIXELS 19
// #define SHOW_NEO_PIXELS 20
// #define SET_NEO_PIXEL 21
// #define CLEAR_ALL_NEO_PIXELS 22
// #define FILL_NEO_PIXELS 23
// #define SPI_INIT 24
// #define SPI_WRITE_BLOCKING 25
// #define SPI_READ_BLOCKING 26
// #define SPI_SET_FORMAT 27
// #define SPI_CS_CONTROL 28
// #define SET_SCAN_DELAY 29
// #define ENCODER_NEW 30
// const int SENSOR_NEW = 31;
// const int GET_ID = 35;
// const int SET_ID = 36;

// #pragma once

enum MESSAGE_IN_TYPE {
  SERIAL_LOOP_BACK = 0,
  SET_PIN_MODE = 1,
  DIGITAL_WRITE = 2,
  PWM_WRITE = 3,
  MODIFY_REPORTING = 4,
  GET_FIRMWARE_VERSION = 5,
  GET_PICO_UNIQUE_ID = 6,
  SERVO_ATTACH = 7,
  SERVO_WRITE = 8,
  SERVO_DETACH = 9, // = unused
  I2C_BEGIN = 10,
  I2C_READ = 11,
  I2C_WRITE = 12,
  SONAR_NEW = 13,
  DHT_NEW = 14,
  STOP_ALL_REPORTS = 15,
  ENABLE_ALL_REPORTS = 16,
  RESET_DATA = 17,
  RESET_BOARD = 18,
  INITIALIZE_NEO_PIXELS = 19,
  SHOW_NEO_PIXELS = 20,
  SET_NEO_PIXEL = 21,
  CLEAR_ALL_NEO_PIXELS = 22,
  FILL_NEO_PIXELS = 23,
  SPI_INIT = 24,
  SPI_WRITE_BLOCKING = 25,
  SPI_READ_BLOCKING = 26,
  SPI_SET_FORMAT = 27,
  SPI_CS_CONTROL = 28,
  SET_SCAN_DELAY = 29,
  ENCODER_NEW = 30,
  SENSOR_NEW = 31,
  GET_ID = 35,
  SET_ID = 36,
  FEATURE_CHECK = 37,
  MAX_IN_MESSAGE_TYPE
};

/*****************************************************
 *                  MESSAGE OFFSETS
 ***************************************************/
// i2c_common
#define I2C_PORT 1
#define I2C_DEVICE_ADDRESS 2 // read and write

// i2c_init
#define I2C_SDA_GPIO_PIN 2
#define I2C_SCL_GPIO_PIN 3

// i2c_read
#define I2C_READ_MESSAGE_ID 3
#define I2C_READ_REGISTER 4
#define I2C_READ_LENGTH 5
#define I2C_READ_NO_STOP_FLAG 6

// I2c_write
#define I2C_WRITE_MESSAGE_ID 3
#define I2C_WRITE_NUMBER_OF_BYTES 4
#define I2C_WRITE_NO_STOP_FLAG 5
#define I2C_WRITE_BYTES_TO_WRITE 6

// This defines how many bytes there are
// that precede the first byte read position
// in the i2c report message buffer.
#define I2C_READ_DATA_BASE_BYTES 6

// Start of i2c data read within the message buffer
#define I2C_READ_START_OF_DATA 7

// Indicator that no i2c register is being specified in the command
#define I2C_NO_REGISTER 254

/******************************* COMMAND BUFFER OFFSETS ************/
// loop back command buffer offsets
#define DATA_TO_LOOP_BACK 1

// set pin mode command buffer offsets
#define SET_PIN_MODE_GPIO_PIN 1
#define SET_PIN_MODE_MODE_TYPE 2

// set pin mode digital input command offsets
#define SET_PIN_MODE_DIGITAL_IN_REPORTING_STATE 3

// set pin mode PWM output input command offsets
#define SET_PIN_MODE_PWM_HIGH_VALUE 2
#define SET_PIN_MODE_PWM_LOW_VALUE 3

// ADC number for temperature sensor
#define ADC_TEMPERATURE_REGISTER 4

// set pin mode analog input command offsets
#define SET_PIN_MODE_ANALOG_IN_REPORTING_STATE 5
#define SET_PIN_MODE_ANALOG_DIFF_HIGH 3
#define SET_PIN_MODE_ANALOG_DIFF_LOW 4

// set pin mode spi offsets
#define SPI_PORT 1
#define SPI_MISO 2
#define SPI_MOSI 3
#define SPI_CLK_PIN 4
#define SPI_FREQ_MSB 5
#define SPI_FREQ_3 6
#define SPI_FREQ_2 7
#define SPI_FREQ_1 8
#define SPI_CS_LIST_LENGTH 9
#define SPI_CS_LIST 10 // beginning of list

// spi write blocking offsets
// #define SPI_PORT 1
#define SPI_WRITE_LEN 2
#define SPI_WRITE_DATA 3

// spi read blocking offsets
// #define SPI_PORT 1
#define SPI_READ_LEN 2
#define SPI_REPEATED_DATA 3

// spi chipselect command offsets
#define SPI_SELECT_PIN 1
#define SPI_SELECT_STATE 2

// spi set format command offsets
// #define SPI_PORT 1
#define SPI_NUMBER_OF_BITS 2
#define SPI_CLOCK_PHASE 3
#define SPI_CLOCK_POLARITY 4

// digital_write
#define DIGITAL_WRITE_GPIO_PIN 1
#define DIGITAL_WRITE_VALUE 2

// pwm write
#define PWM_WRITE_GPIO_PIN 1
#define PWM_WRITE_HIGH_VALUE 2
#define PWM_WRITE_LOW_VALUE 3

// modify reporting
// This can be a gpio pin or adc channel
#define MODIFY_REPORTING_PIN 2
#define MODIFY_REPORTING_TYPE 1

// i2c report buffer offsets
#define I2C_PACKET_LENGTH 0
#define I2C_REPORT_ID 1
#define I2C_REPORT_PORT 2
#define I2C_REPORT_DEVICE_ADDRESS 3
#define I2C_REPORT_READ_REGISTER 4
#define I2C_REPORT_READ_NUMBER_DATA_BYTES 5

#define I2C_ERROR_REPORT_LENGTH 4
#define I2C_ERROR_REPORT_NUM_OF_BYTE_TO_SEND 5

// spi report buffer offsets
#define SPI_PACKET_LENGTH 0
#define SPI_REPORT_ID 1
#define SPI_REPORT_PORT 2
#define SPI_REPORT_NUMBER_OF_DATA_BYTES 3
#define SPI_DATA 4

#define SPI_READ_DATA_BASE_BYTES 5

// scan delay buffer offsets
#define SCAN_DELAY 1

// init neopixels
// command offsets
#define NP_PIN_NUMBER 1
#define NP_NUMBER_OF_PIXELS 2
#define NP_RED_FILL 3
#define NP_GREEN_FILL 4
#define NP_BLUE_FILL 5

// NeoPixel clock frequency
#define NP_FREQUENCY 800000

// Pixel buffer array offsets
#define RED 0
#define GREEN 1
#define BLUE 2

// set_neo_pixel command offsets
#define NP_PIXEL_NUMBER 1
#define NP_SET_RED 2
#define NP_SET_GREEN 3
#define NP_SET_BLUE 4
#define NP_SET_AUTO_SHOW 5

// fill_neo_pixels command offsets
#define NP_FILL_RED 1
#define NP_FILL_GREEN 2
#define NP_FILL_BLUE 3
#define NP_FILL_AUTO_SHOW 4

// clear_all_neo_pixels command offsets
#define NP_CLEAR_AUTO_SHOW 1

#define MAXIMUM_NUM_NEOPIXELS 150

// init hc-sr04
// command offsets
#define SONAR_TRIGGER_PIN 1
#define SONAR_ECHO_PIN 2

/* Maximum number of Sonar devices */
#define MAX_SONARS 4

// init dht command offsets
#define DHT_DATA_PIN 1

/* Maximum number of DHT devices */
#define MAX_DHTS 2

/* maximum dht timings */
const uint DHT_MAX_TIMINGS = 85;

// encoder init
#define ENCODER_TYPE 1
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3

// Max encoder devices
#define MAX_ENCODERS 4

typedef enum { SINGLE = 1, QUADRATURE = 2 } ENCODER_TYPES;

// encoder reports are identified by pin A
#define ENCODER_REPORT_PIN_A 2
#define ENCODER_REPORT_STEP 3

/*********************** REPORTING BUFFER OFFSETS ******************/
// loopback buffer offset for data being looped back
#define LOOP_BACK_DATA 2

// send_debug message buffer offsets
#define DEBUG_ID 2
#define DEBUG_VALUE_HIGH_BYTE 3
#define DEBUG_VALUE_LOW_BYTE 4

// digital input report buffer offsets
#define DIGITAL_INPUT_GPIO_PIN 2
#define DIGITAL_INPUT_GPIO_VALUE 3

// analog input report buffer offsets
#define ANALOG_INPUT_GPIO_PIN 2
#define ANALOG_VALUE_HIGH_BYTE 3
#define ANALOG_VALUE_LOW_BYTE 4

// sonar report buffer offsets
#define SONAR_TRIG_PIN 2
#define M_WHOLE_VALUE 3
#define CM_WHOLE_VALUE 4

// dht report buffer offset
#define DHT_REPORT_PIN 2
#define DHT_HUMIDITY_WHOLE_VALUE 3
#define DHT_HUMIDITY_FRAC_VALUE 4
#define DHT_TEMPERATURE_WHOLE_VALUE 5
#define DHT_TEMPERATURE_FRAC_VALUE 6

/******************************************************
 *                 PIN MODE DEFINITIONS
 *****************************************************/

enum PIN_MODES {
  NOT_SET = 255,
  INPUT = 0,
  OUTPUT = 1,
  PWM = 2,
  INPUT_PULL_UP = 3,
  INPUT_PULL_DOWN = 4,
  ANALOG_INPUT = 5,
  SONAR_MODE = 7,
  DHT_MODE = 8
};
#define PIN_MODE_NOT_SET 255

/**************************************************
 *               REPORT DEFINITIONS
 **************************************************/
// #define SERIAL_LOOP_BACK_REPORT 0
// #define DIGITAL_REPORT 2
// #define ANALOG_REPORT 3
// #define FIRMWARE_REPORT 5
// #define REPORT_PICO_UNIQUE_ID 6
// #define SERVO_UNAVAILABLE 7 // for the future
// #define I2C_WRITE_REPORT 8
// #define I2C_READ_FAILED 9
// #define I2C_READ_REPORT 10
// #define SONAR_DISTANCE 11
// #define DHT_REPORT 12
// #define SPI_REPORT 13
// #define ENCODER_REPORT 14
// #define DEBUG_PRINT 99
// const int SENSOR_REPORT = 20;
// const int PONG_REPORT = 32;
// const int MODULE_REPORT = 34;

enum MESSAGE_OUT_TYPE {
  SERIAL_LOOP_BACK_REPORT = 0,
  DIGITAL_REPORT = 2,
  ANALOG_REPORT = 3,
  FIRMWARE_REPORT = 5,
  REPORT_PICO_UNIQUE_ID = 6,
  SERVO_UNAVAILABLE = 7, // for the future
  I2C_WRITE_REPORT = 8,
  I2C_READ_FAILED = 9,
  I2C_READ_REPORT = 10,
  SONAR_DISTANCE = 11,
  DHT_REPORT = 12,
  SPI_REPORT = 13,
  ENCODER_REPORT = 14,
  DEBUG_PRINT = 99,
  SENSOR_REPORT = 20,
  PONG_REPORT = 32,
  MODULE_REPORT = 34,
  MAX_OUT_MESSAGE_TYPE
};

/***************************************************************
 *          INPUT PIN REPORTING CONTROL SUB COMMANDS
 ***************************************************************/
#define REPORTING_DISABLE_ALL 0
#define REPORTING_ANALOG_ENABLE 1
#define REPORTING_DIGITAL_ENABLE 2
#define REPORTING_ANALOG_DISABLE 3
#define REPORTING_DIGITAL_DISABLE 4

/* Maximum Supported pins */
#define MAX_DIGITAL_PINS_SUPPORTED 30
#define MAX_ANALOG_PINS_SUPPORTED 5

/* Firmware Version Values */
#define FIRMWARE_MAJOR 1
#define FIRMWARE_MINOR 3

// maximum length of a command packet in bytes
#define MAX_COMMAND_LENGTH 30

// Indicator that no i2c register is being specified in the command
#define I2C_NO_REGISTER_SPECIFIED 254