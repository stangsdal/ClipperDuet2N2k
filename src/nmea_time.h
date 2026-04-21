#pragma once

#include <stdint.h>

class tN2kMsg;

extern uint8_t SID;
extern uint16_t DaysSince1970;
extern double SecondsSinceMidnight;

void TimeUpdate();
void HandleNMEA2000Msg(const tN2kMsg& N2kMsg);
