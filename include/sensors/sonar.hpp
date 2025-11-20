#pragma once

#include "message_types.hpp"
#include "pico/mutex.h"
#include "pico/time.h"
#include <cstdint>
void sonar_new();

// This structure describes an HC-SR04 type device
typedef struct hc_sr04_descriptor {
  uint trig_pin; // trigger pin
  uint echo_pin; // echo pin
  uint32_t start_time;
  uint32_t last_time_diff;
  int last_dist;
  bool single_pin;
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

// number of active sonars
extern int sonar_count;
extern uint sonar_offset;

// sonar device descriptors
extern sonar_data the_hc_sr04s;
void sonar_callback(uint gpio, uint32_t events);

bool sonar_timer_callback(repeating_timer_t *rt);

void scan_sonars();