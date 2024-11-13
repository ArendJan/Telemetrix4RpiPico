#pragma once
#include <pico/types.h>
#include <vector>

extern void serial_write(const int *buffer, int num_of_bytes_to_send);

void serial_write(std::vector<uint8_t> data);

// functions to allow uart loopback on the mirte-master pcb for the servos
int get_byte();

void put_byte(uint8_t byte);
