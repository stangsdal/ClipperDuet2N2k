#pragma once

#include <Arduino.h>

struct ClipperLCD {
  uint8_t digit0;
  uint8_t digit1;
  uint8_t digit2;
  uint8_t digit3;
  uint8_t digit4;
  uint8_t digit5;
  uint8_t digit6;
  uint8_t info1;
  uint8_t info2;
};

struct ClipperData {
  double speed;          // speed in m/s
  double depth;          // depth in m
  double shallow_alarm;  // in m
  double speed_alarm;    // in m/s
  double total;          // in m (accumulated by unit)
  double trip;           // in m (since power-on)

  unsigned long last_depth;
  unsigned long last_speed;
  unsigned long last_total;
  unsigned long last_trip;

  double offset;     // keel offset in m
  double threshold;  // depth threshold in m
  double cal;        // speed calibration in %
};

extern ClipperLCD clipperlcd;
extern ClipperData clipperdata;
