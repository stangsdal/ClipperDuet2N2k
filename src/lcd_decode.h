#pragma once

#include <Arduino.h>

constexpr uint32_t BUFFER_SIZE = 36;

extern uint8_t spi_slave_rx_buf[BUFFER_SIZE];

void buf2clipperlcd();
uint32_t digits2int(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, bool dot);
double rowa2double();
double rowb2double();
void printBits(void const* ptr, size_t size);
