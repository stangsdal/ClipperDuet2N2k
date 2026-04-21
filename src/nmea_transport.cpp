#include "nmea_transport.h"

#include <Arduino.h>
#include <N2kMessages.h>
#include <NMEA2000.h>
#include <NMEA0183Msg.h>
#include <version.h>

#ifndef N2K_SOFTWARE_VERSION
#define N2K_SOFTWARE_VERSION "0.0.0.0 (#__DATE__)"
#endif
#ifndef GIT_DESCRIBE
#define GIT_DESCRIBE "unknown"
#endif

// From https://github.com/ttlappalainen/NMEA2000/blob/master/Examples/ESP32/NMEA2000ToWiFiAsSeaSmart/BoardSerialNumber.cpp
static uint32_t GetSerialNumber() {
#if defined(ARDUINO_ARCH_ESP32)
  uint8_t chipid[6];
  esp_efuse_mac_get_default(chipid);
  return chipid[0] + (chipid[1] << 8) + (chipid[2] << 16) + (chipid[3] << 24);
#else
  return 999999;
#endif
}

void InitNMEA2000Transport(tNMEA2000& nmea2000,
                           const unsigned long* transmit_messages,
                           const unsigned long* receive_messages,
                           void (*msg_handler)(const tN2kMsg&),
                           const char* installation_description2) {
  nmea2000.SetN2kCANMsgBufSize(8);
  nmea2000.SetN2kCANReceiveFrameBufSize(200);

  char SnoStr[33];
  uint32_t SerialNumber = GetSerialNumber();
  snprintf(SnoStr, 32, "%lu", (long unsigned int)SerialNumber);

  nmea2000.SetProductInformation(SnoStr,            // Manufacturer's Model serial code
                                 1337,              // Manufacturer's product code
                                 "ClipperDuet2N2k", // Manufacturer's Model ID
                                 N2K_SOFTWARE_VERSION,
                                 "1.0.0.0 (1998-01-01)");

  nmea2000.SetDeviceInformation(SerialNumber, 60, 135, 275);
  nmea2000.SetInstallationDescription1("ClipperDuet2N2k " GIT_DESCRIBE
                                       " by Soenke J. Peters");
  nmea2000.SetInstallationDescription2(installation_description2);

  // Serial is used by SensESP for logging; disable NMEA2000 Actisense forwarding
  nmea2000.EnableForward(false);
  nmea2000.SetMode(tNMEA2000::N2km_ListenAndNode, 32);

  nmea2000.ExtendTransmitMessages(transmit_messages);
  nmea2000.ExtendReceiveMessages(receive_messages);
  nmea2000.SetMsgHandler(msg_handler);

  nmea2000.Open();
}

bool NMEA0183SetVLW(tNMEA0183Msg& NMEA0183Msg, double TotalLog, double TripLog,
                    const char* Src) {
  if (!NMEA0183Msg.Init("VLW", Src)) return false;
  if (!NMEA0183Msg.AddDoubleField(TotalLog / 1852.0)) return false;
  if (!NMEA0183Msg.AddStrField("N")) return false;
  if (!NMEA0183Msg.AddDoubleField(TripLog / 1852.0)) return false;
  if (!NMEA0183Msg.AddStrField("N")) return false;
  return true;
}
