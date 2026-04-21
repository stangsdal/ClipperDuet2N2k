#pragma once

#include <stdint.h>

class tN2kMsg;
class tNMEA0183Msg;
class tNMEA2000;

void InitNMEA2000Transport(tNMEA2000& nmea2000,
                           const unsigned long* transmit_messages,
                           const unsigned long* receive_messages,
                           void (*msg_handler)(const tN2kMsg&),
                           const char* installation_description2);

bool NMEA0183SetVLW(tNMEA0183Msg& NMEA0183Msg, double TotalLog, double TripLog,
                    const char* Src = "II");
