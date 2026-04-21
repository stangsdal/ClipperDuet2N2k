/*
  NASA Clipper Duet Echo Sounder/Log to NMEA2000 and Signal K converter (ClipperDuet2N2k)
  2023-01-30 by Soenke J. Peters
  Rewritten for the SensESP platform 2026

  This code reads the display data from the ht1621 lcd driver of the NASA Clipper Duet
  and converts it to useful values sent over the NMEA2000 network and to a Signal K server
  via the SensESP framework.
*/

// SPDX-License-Identifier: MIT

#if defined(ARDUINO_ARCH_ESP8266)
#error "ESP8266 (ESP-12F) is not supported by this firmware. It requires ESP32SPISlave, ESP32 Preferences/NVS and ESP32 NMEA2000 CAN support."
#endif

/*
 Timeout in s for Trip and Total distance
 if more than this time has elapsed between the display of the two values,
 the Trip record is considered invalid and therefore no distance log values are sent
 */
#ifndef DISTANCE_TIMEOUT
#define DISTANCE_TIMEOUT 60
#endif
// Consider this a safe sensor to keel distance (in m) in case the offset has not been read from the device
// This is a positive value
#ifndef SAFE_OFFSET
#define SAFE_OFFSET 3
#endif

// Pin mappings for LCD data lines to SPI pins of the ESP32
#ifndef PIN_HTDATA
#define PIN_HTDATA GPIO_NUM_12    // HT1621 DATA is SPI MOSI on our ESP32 SPI Slave implementation
#endif
#ifndef PIN_HTDATAOUT
#define PIN_HTDATAOUT GPIO_NUM_13 // unused, but must a usable pin for SPI MISO
#endif
#ifndef PIN_HTCLK
#define PIN_HTCLK GPIO_NUM_14     // HT1621 WR is SPI Clk on our ESP32 SPI Slave implementation
#endif
#ifndef PIN_HTCS
#define PIN_HTCS GPIO_NUM_27      // HT1621 CS is SPI CS on our ESP32 SPI Slave implementation
#endif

// Pins where the CAN bus transceiver is attached
#ifndef ESP32_CAN_TX_PIN
#define ESP32_CAN_TX_PIN GPIO_NUM_5
#endif
#ifndef ESP32_CAN_RX_PIN
#define ESP32_CAN_RX_PIN GPIO_NUM_4
#endif

// Baud rate for the NMEA0183 output on Serial2 (usually 4800 or 38400 bps)
#ifndef NMEA0183_SPEED
#define NMEA0183_SPEED 4800
#endif

// Pins for Serial2; use ESP32 defaults when not overridden
#ifndef SERIAL2_TXD
#define SERIAL2_TXD (-1)
#endif
#ifndef SERIAL2_RXD
#define SERIAL2_RXD (-1)
#endif

// Older Platformio ESP32 versions need this
//#define OLD_SPI_COMMS

// Have some printf() status messages on the serial console.
//#define DEBUG

/* *********************************************************************************************
  No user-configurable stuff below
*/

#include <version.h>

#ifndef N2K_SOFTWARE_VERSION
#warning "no N2K_SOFTWARE_VERSION from git"
#define N2K_SOFTWARE_VERSION "0.0.0.0 (#__DATE__)"
#endif
#ifndef GIT_DESCRIBE
#warning "no GIT_DESCRIBE from git"
#define GIT_DESCRIBE "unknown"
#endif

// define for debugging communications with the HT1621 LCD controller (print raw received data, not useful for production)
// #define DEBUG_COMMS

#ifdef DEBUG_COMMS
#ifndef DEBUG
#define DEBUG
#endif
#endif
#ifdef DEBUG
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

// SensESP framework – handles WiFi, Signal K server connection, OTA, web config UI
#include "sensesp_app_builder.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
using namespace sensesp;

#include <Arduino.h>
#include <Preferences.h>

// See https://github.com/ttlappalainen/NMEA2000/
#include <NMEA2000_CAN.h> // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
// See https://github.com/hideakitai/ESP32SPISlave
#include <ESP32SPISlave.h>

#include "clipper_data.h"
#include "lcd_decode.h"
#include "nmea_time.h"
#include "nmea_transport.h"

Preferences preferences;
ESP32SPISlave slave;

#include <HardwareSerial.h>
#include <NMEA0183.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Messages.h>
tNMEA0183 NMEA0183_Out;

unsigned long now;

// Set the information for other bus devices, which messages we support
const unsigned long ReceiveMessages[] PROGMEM = {129029L, // GNSS (used for time)
                                                 126992L, // System time
                                                 65361L,  // Seatalk: Silence Alarm
                                                 0};
const unsigned long TransmitMessages[] PROGMEM = {128275L, // Distance Log
                                                  128259L, // Boat speed
                                                  128267L, // Depth
                                                  65288L,  // Seatalk: Alarm
                                                  0};

#define PP_STRINGIFY_IMPL(X) #X
#define PP_STRINGIFY(X) PP_STRINGIFY_IMPL(X)

#define PINDESCRIPTION "MOSI(HT_DATA)=" PP_STRINGIFY(PIN_HTDATA) ",MISO=" PP_STRINGIFY(PIN_HTDATAOUT) ",CLK(HT_WR)= " PP_STRINGIFY(PIN_HTCLK) ",CS=" PP_STRINGIFY(PIN_HTCS) ",CAN_TX=" PP_STRINGIFY(ESP32_CAN_TX_PIN) ",CAN_RX=" PP_STRINGIFY(ESP32_CAN_RX_PIN)

// bit field to queue the saves to NVM
// speed_alarm | shallow_alarm | cal | threshold | offset
const uint32_t BF_OFFSET = 1;
const uint32_t BF_THRESHOLD = 1 << 1;
const uint32_t BF_CAL = 1 << 2;
const uint32_t BF_SHALLOW_ALARM = 1 << 3;
const uint32_t BF_SPEED_ALARM = 1 << 4;

uint32_t queue_save = 0;
tN2kMsg N2kMsg;
tNMEA0183Msg NMEA0183Msg;

/*
 Process one SPI polling cycle: queue a receive buffer if empty, then consume all
 available frames.  This function is called from a fast ReactESP repeat event (10 ms)
 and contains the same display-decoding logic that was previously in loop().
 Parsed values are written to clipperdata so that the SensESP RepeatSensors can
 read them and forward them to the Signal K server.
*/
void process_spi()
{
  now = millis();
  TimeUpdate();

  NMEA2000.ParseMessages();

  if (slave.remained() == 0)
  {
    slave.queue(spi_slave_rx_buf, BUFFER_SIZE);
  }

  while (slave.available())
  {
    uint32_t num = slave.size();

    if (num > 2 && num < BUFFER_SIZE)
    {
      if ((spi_slave_rx_buf[0] & 0b11100000) == 0b10100000)
      {
        // LCD Write to display memory
#ifdef DEBUG_COMMS
        printf("%d Mem: ", num);
#else
        buf2clipperlcd();

        if ((clipperlcd.info2 & (1 << 2)) == (1 << 2))
        {
          // Line LCD segment is displayed – unit is not in settings mode

          if (queue_save > 0)
          {
            if ((queue_save & BF_SHALLOW_ALARM) == BF_SHALLOW_ALARM)
            {
              preferences.putDouble("shallow_alarm", clipperdata.shallow_alarm);
              DEBUG_PRINT("shallow_alarm saved: %fm\n", clipperdata.shallow_alarm);
            }
            if ((queue_save & BF_SPEED_ALARM) == BF_SPEED_ALARM)
            {
              preferences.putDouble("speed_alarm", clipperdata.speed_alarm);
              DEBUG_PRINT("speed_alarm saved: %fm\n", clipperdata.speed_alarm);
            }
            if ((queue_save & BF_OFFSET) == BF_OFFSET)
            {
              preferences.putDouble("offset", clipperdata.offset);
              DEBUG_PRINT("Setting offset saved: %fm\n", clipperdata.offset);
            }
            if ((queue_save & BF_THRESHOLD) == BF_THRESHOLD)
            {
              preferences.putDouble("threshold", clipperdata.threshold);
              DEBUG_PRINT("Setting threshold saved: %fm\n", clipperdata.threshold);
            }
            if ((queue_save & BF_CAL) == BF_CAL)
            {
              preferences.putDouble("cal", clipperdata.cal);
              DEBUG_PRINT("Setting cal saved: %fm\n", clipperdata.cal);
            }
            queue_save = 0;
          }

          SID = (++SID) & 0xff;

          double depth = rowb2double();
          if (depth != N2kDoubleNA)
          {
            if ((clipperlcd.info2 & 1) == 1)
            {
              clipperdata.depth = depth;
              clipperdata.last_depth = now;
            }
            else if ((clipperlcd.info2 & (1 << 1)) == (1 << 1))
            {
              clipperdata.depth = depth / 3.281;
              clipperdata.last_depth = now;
            }
            else
            {
              clipperdata.depth = N2kDoubleNA;
            }
          }

          SetN2kWaterDepth(N2kMsg, SID, clipperdata.depth, clipperdata.offset, N2kDoubleNA);
          NMEA2000.SendMsg(N2kMsg);

          NMEA0183SetDPT(NMEA0183Msg, clipperdata.depth, clipperdata.offset);
          NMEA0183_Out.SendMessage(NMEA0183Msg);

          double speed = N2kDoubleNA;
          double distance = -1;

          if (clipperlcd.info1 < (1 << 6))
          {
            speed = rowa2double();
            if (speed != N2kDoubleNA)
            {
              if ((clipperlcd.info1 & 0b00100000) == 0b00100000)
              {
                clipperdata.speed = speed * 0.514444444444;
              }
              else if ((clipperlcd.info1 & 0b00001100) == 0b00001100)
              {
                clipperdata.speed = speed / 3.6;
              }
              else if ((clipperlcd.info1 & 0b00010000) == 0b00010000)
              {
                clipperdata.speed = speed / 2.237;
              }
              else
              {
                clipperdata.speed = 0.0;
              }
              clipperdata.last_speed = now;
            }
            else
            {
              clipperdata.speed = N2kDoubleNA;
            }

            SetN2kBoatSpeed(N2kMsg, SID, clipperdata.speed, N2kDoubleNA, N2kSWRT_Paddle_wheel);
            NMEA2000.SendMsg(N2kMsg);

            NMEA0183SetVHW(NMEA0183Msg, NMEA0183DoubleNA, NMEA0183DoubleNA, clipperdata.speed);
            NMEA0183_Out.SendMessage(NMEA0183Msg);

            DEBUG_PRINT("Speed: %fm/s, Depth: %fm\n", clipperdata.speed, clipperdata.depth);
          }
          else
          {
            distance = rowa2double();
            if (distance != N2kDoubleNA)
            {
              if ((clipperlcd.info1 & 0b00000011) == 0b00000011)
              {
                distance *= 1852;
              }
              else if ((clipperlcd.info1 & 0b00001000) == 0b00001000)
              {
                distance *= 1000;
              }
              else if ((clipperlcd.info1 & 0b00000001) == 0b00000001)
              {
                distance *= 1609;
              }
              else
              {
                distance = N2kDoubleNA;
              }
            }

            if (distance != N2kDoubleNA)
            {
              if ((clipperlcd.info1 & 0b10000000) == 0b10000000)
              {
                clipperdata.trip = distance;
                clipperdata.last_trip = now;

                SetN2kDistanceLog(N2kMsg, DaysSince1970, SecondsSinceMidnight, clipperdata.total, clipperdata.trip);
                NMEA2000.SendMsg(N2kMsg);

                NMEA0183SetVLW(NMEA0183Msg, clipperdata.total, clipperdata.trip);
                NMEA0183_Out.SendMessage(NMEA0183Msg);

                DEBUG_PRINT("Trip: %fm, Total: %fm, DaysSince1970: %u, SecondsSinceMidnight: %f\n",
                            clipperdata.trip, clipperdata.total, DaysSince1970, SecondsSinceMidnight);
              }
              else if ((clipperlcd.info1 & 0b01000000) == 0b01000000)
              {
                clipperdata.total = distance;
                clipperdata.last_total = now;

                if ((now - clipperdata.last_trip) < (DISTANCE_TIMEOUT * 1000))
                {
                  SetN2kDistanceLog(N2kMsg, DaysSince1970, SecondsSinceMidnight, clipperdata.total, clipperdata.trip);
                  NMEA2000.SendMsg(N2kMsg);

                  NMEA0183SetVLW(NMEA0183Msg, clipperdata.total, clipperdata.trip);
                  NMEA0183_Out.SendMessage(NMEA0183Msg);

                  DEBUG_PRINT("Total: %fm, Trip: %fm, DaysSince1970: %u, SecondsSinceMidnight: %f\n",
                              clipperdata.total, clipperdata.trip, DaysSince1970, SecondsSinceMidnight);
                }
                else
                {
                  DEBUG_PRINT("Total: %fm\n", clipperdata.total);
                }
              }
            }
          }
        }
        else
        {
          // Settings dialogues
          if (clipperlcd.digit0 == 'W' && clipperlcd.digit1 == '(') // "u_underline Con"
          {
            double offset = rowb2double();
            if (offset != N2kDoubleNA)
            {
              if ((clipperlcd.info2 & 1) == 1)
              {
                offset = -offset;
              }
              else if ((clipperlcd.info2 & (1 << 1)) == (1 << 1))
              {
                offset = -(offset / 3.281);
              }
              else
              {
                offset = -SAFE_OFFSET;
              }
              if (offset != clipperdata.offset)
              {
                clipperdata.offset = offset;
                queue_save |= BF_OFFSET;
              }
            }
          }
          else if (clipperlcd.digit0 == 'T' && clipperlcd.digit1 == '(') // "t Con"
          {
            double threshold = rowb2double();
            if (threshold != N2kDoubleNA)
            {
              if ((clipperlcd.info2 & (1 << 1)) == (1 << 1))
              {
                threshold = (threshold / 3.281);
              }
              else if (!((clipperlcd.info2 & 1) == 1))
              {
                threshold = 0.0;
              }
              if (threshold != clipperdata.threshold)
              {
                clipperdata.threshold = threshold;
                queue_save |= BF_THRESHOLD;
              }
            }
          }
          else if (clipperlcd.digit4 == '(' && ((clipperlcd.info2 & 0b100011) == 0))
          {
            double cal = rowa2double();
            if (cal != N2kDoubleNA && cal != clipperdata.cal)
            {
              clipperdata.cal = cal;
              queue_save |= BF_CAL;
            }
          }
          else if (clipperlcd.digit1 == '5' && clipperlcd.digit2 == 'X') // "SHA"
          {
            double shallow_alarm = rowb2double();
            if (shallow_alarm != N2kDoubleNA)
            {
              if ((clipperlcd.info2 & 1) == 1)
              {
                clipperdata.shallow_alarm = shallow_alarm;
              }
              else if ((clipperlcd.info2 & (1 << 1)) == (1 << 1))
              {
                clipperdata.shallow_alarm = shallow_alarm / 3.281;
              }
              else
              {
                clipperdata.shallow_alarm = SAFE_OFFSET + 1;
              }
              queue_save |= BF_SHALLOW_ALARM;
            }
          }
          else if (clipperlcd.digit4 == '5' && clipperlcd.digit5 == 'P') // "SPd"
          {
            double speed_alarm = rowa2double();
            if (speed_alarm != N2kDoubleNA)
            {
              if ((clipperlcd.info1 & 0b00100000) == 0b00100000)
              {
                clipperdata.speed_alarm = speed_alarm * 0.514444444444;
              }
              else if ((clipperlcd.info1 & 0b00001100) == 0b00001100)
              {
                clipperdata.speed_alarm = speed_alarm / 3.6;
              }
              else if ((clipperlcd.info1 & 0b00010000) == 0b00010000)
              {
                clipperdata.speed_alarm = speed_alarm / 2.237;
              }
            }
            else
            {
              clipperdata.speed_alarm = 0.0;
            }
            queue_save |= BF_SPEED_ALARM;
          }
          else
          {
            DEBUG_PRINT("?: %c %c%c%c %c%c%c\n", clipperlcd.digit0, clipperlcd.digit1,
                        clipperlcd.digit2, clipperlcd.digit3, clipperlcd.digit4,
                        clipperlcd.digit5, clipperlcd.digit6);
          }
        }
#endif
      }
#ifdef DEBUG_COMMS
      else if ((spi_slave_rx_buf[0] & 0b11100000) == 0b10000000)
      {
        printf("%d, Command: ", num);
      }
      printBits(spi_slave_rx_buf, num);
      printf("\n");
#endif
    }

    slave.pop();
  }

  // Drain Serial2 input to avoid buffer overflow
  while (Serial2.available())
  {
    Serial2.read();
  }
}

// Helper: convert a double in N2k base units to float for Signal K, returning NaN when invalid
static inline float sk_float(double v)
{
  return (v != N2kDoubleNA) ? static_cast<float>(v) : NAN;
}

static double pref_double_or_default(Preferences& prefs, const char* key,
                                     double default_value)
{
  return prefs.isKey(key) ? prefs.getDouble(key) : default_value;
}

void setup()
{
  // SensESP initialises Serial for logging; do this first
  SetupLogging();

  clipperdata.depth = N2kDoubleNA;
  preferences.begin("ClipperDuet2N2k", false);
    clipperdata.offset = pref_double_or_default(preferences, "offset", -SAFE_OFFSET);
    clipperdata.cal = pref_double_or_default(preferences, "cal", 100);
    clipperdata.threshold = pref_double_or_default(preferences, "threshold", 0.0);
    clipperdata.shallow_alarm =
      pref_double_or_default(preferences, "shallow_alarm", 0.0);
    clipperdata.speed_alarm =
      pref_double_or_default(preferences, "speed_alarm", 0.0);

  // Build the SensESP application.
  // SensESP manages WiFi connectivity, OTA firmware updates, and the Signal K
  // server connection. The web config UI is available at http://clipperduet2n2k.local/
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
      ->set_hostname("clipperduet2n2k")
      // .set_sk_server("192.168.10.3", 80)
      // .set_ota_password("my_ota_password")
      ->get_app();

  InitNMEA2000Transport(NMEA2000, TransmitMessages, ReceiveMessages,
                        HandleNMEA2000Msg, PINDESCRIPTION);

  // NMEA0183 output always goes to Serial2
  if (SERIAL2_TXD == -1)
  {
    Serial2.begin(NMEA0183_SPEED);
  }
  else
  {
    Serial2.begin(NMEA0183_SPEED, SERIAL2_RXD, SERIAL2_TXD);
  }
  NMEA0183_Out.SetMessageStream(&Serial2);
  NMEA0183_Out.Open();

  ESP_LOGI("ClipperDuet2N2k", "Starting %s", GIT_DESCRIBE);

  slave.setDataMode(SPI_MODE3);
  slave.begin(HSPI, PIN_HTCLK, PIN_HTDATAOUT, PIN_HTDATA, PIN_HTCS);
  memset(spi_slave_rx_buf, 0, BUFFER_SIZE);

  // Fast event: poll HT1621 SPI bus and parse LCD data every 10 ms.
  // This also calls NMEA2000.ParseMessages() and sends NMEA2000 / NMEA0183 messages.
  event_loop()->onRepeat(10, []() { process_spi(); });

  // Signal K outputs – values are read from clipperdata and sent to the SK server.
  // The config paths exposed at /config allow the SK path to be changed at runtime.

  auto* depth_sensor = new RepeatSensor<float>(
      1000,
      []() { return sk_float(clipperdata.depth); });
  depth_sensor->connect_to(
      new SKOutputFloat("environment.depth.belowKeel", "/sensors/depth/sk"));

  auto* speed_sensor = new RepeatSensor<float>(
      1000,
      []() { return sk_float(clipperdata.speed); });
  speed_sensor->connect_to(
      new SKOutputFloat("navigation.speedThroughWater", "/sensors/speed/sk"));

  auto* total_sensor = new RepeatSensor<float>(
      5000,
      []() { return sk_float(clipperdata.total); });
  total_sensor->connect_to(
      new SKOutputFloat("navigation.log", "/sensors/total/sk"));

  auto* trip_sensor = new RepeatSensor<float>(
      5000,
      []() { return sk_float(clipperdata.trip); });
  trip_sensor->connect_to(
      new SKOutputFloat("navigation.trip.log", "/sensors/trip/sk"));

  sensesp_app->start();
}

void loop()
{
  event_loop()->tick();
}
