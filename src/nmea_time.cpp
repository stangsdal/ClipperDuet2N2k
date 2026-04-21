#include "nmea_time.h"

#include <Arduino.h>
#include <N2kMessages.h>

uint8_t SID = 0;
uint16_t DaysSince1970 = 0;
double SecondsSinceMidnight = 0.0;

static unsigned long received_systemtime = 0;
static uint16_t received_DaysSince1970 = 0;
static double received_SecondsSinceMidnight = 0.0;

static bool ParseN2kPGN65361(const tN2kMsg& N2kMsg, uint8_t& alarm_id,
                             uint8_t& alarm_group, uint32_t& reserved_field) {
  if (N2kMsg.PGN != 65361L) {
    return false;
  }

  int Index = 0;
  alarm_id = N2kMsg.GetByte(Index);
  alarm_group = N2kMsg.GetByte(Index);
  reserved_field = N2kMsg.Get2ByteUInt(Index);

  return true;
}

static bool ParseN2kSeatalkSilenceAlarm(const tN2kMsg& N2kMsg, uint8_t& alarm_id,
                                        uint8_t& alarm_group,
                                        uint32_t& reserved_field) {
  return ParseN2kPGN65361(N2kMsg, alarm_id, alarm_group, reserved_field);
}

// PGN129029 handler
static void handle_GNSS(const tN2kMsg& N2kMsg) {
  unsigned char local_sid;
  double Latitude;
  double Longitude;
  double Altitude;
  tN2kGNSStype GNSStype;
  tN2kGNSSmethod GNSSmethod;
  unsigned char nSatellites;
  double HDOP;
  double PDOP;
  double GeoidalSeparation;
  unsigned char nReferenceStations;
  tN2kGNSStype ReferenceStationType;
  uint16_t ReferenceSationID;
  double AgeOfCorrection;

  if (ParseN2kGNSS(N2kMsg, local_sid, DaysSince1970, SecondsSinceMidnight,
                   Latitude, Longitude, Altitude, GNSStype, GNSSmethod,
                   nSatellites, HDOP, PDOP, GeoidalSeparation,
                   nReferenceStations, ReferenceStationType, ReferenceSationID,
                   AgeOfCorrection)) {
    received_systemtime = millis();
    received_DaysSince1970 = DaysSince1970;
    received_SecondsSinceMidnight = SecondsSinceMidnight;
  }
}

// PGN126992 handler
static void handle_SystemTime(const tN2kMsg& N2kMsg) {
  unsigned char tSID;
  tN2kTimeSource TimeSource;

  if (ParseN2kSystemTime(N2kMsg, tSID, DaysSince1970, SecondsSinceMidnight,
                         TimeSource)) {
    received_systemtime = millis();
    received_DaysSince1970 = DaysSince1970;
    received_SecondsSinceMidnight = SecondsSinceMidnight;
  }
}

static void handle_SeatalkSilenceAlarm(const tN2kMsg& N2kMsg) {
  uint8_t alarm_id;
  uint8_t alarm_group;
  uint32_t reserved_field;

  if (ParseN2kSeatalkSilenceAlarm(N2kMsg, alarm_id, alarm_group,
                                  reserved_field)) {
    if (alarm_group == 0) {
      if (alarm_id == 1) {
      } else if (alarm_id == 2) {
      }
    }
  }
}

void TimeUpdate() {
  const unsigned long deltat_s = (millis() - received_systemtime) / 1000;
  DaysSince1970 = received_DaysSince1970 + (deltat_s / (24 * 3600));
  SecondsSinceMidnight =
      received_SecondsSinceMidnight + (deltat_s % (24 * 3600));
}

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg& N2kMsg);
} tNMEA2000Handler;

static tNMEA2000Handler NMEA2000Handlers[] = {
    {126992L, &handle_SystemTime},
    {129029L, &handle_GNSS},
    {65361L, &handle_SeatalkSilenceAlarm},
    {0, nullptr}};

void HandleNMEA2000Msg(const tN2kMsg& N2kMsg) {
  int iHandler;
  for (iHandler = 0;
       NMEA2000Handlers[iHandler].PGN != 0 &&
       !(N2kMsg.PGN == NMEA2000Handlers[iHandler].PGN);
       iHandler++) {
  }
  if (NMEA2000Handlers[iHandler].PGN != 0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg);
  }
}
