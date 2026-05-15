#include <Arduino.h>
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>

#include <HAMLET_ClipperDuet.h>

using hamlet::clipperduet::Decoder;
using hamlet::clipperduet::Event;

// Replace this with real SPI capture from HT1621 bus.
static bool ReadClipperFrame(uint8_t* out, size_t* out_size) {
  (void)out;
  (void)out_size;
  return false;
}

Decoder decoder;
uint8_t sid = 0;

void setup() {
  // HALMET project should keep its existing NMEA2000 setup here.
  // Example only:
  // NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 22);
  // NMEA2000.Open();
}

void loop() {
  NMEA2000.ParseMessages();

  uint8_t frame[hamlet::clipperduet::kFrameBufferSize] = {0};
  size_t frame_size = 0;

  if (!ReadClipperFrame(frame, &frame_size)) {
    return;
  }

  const Event event = decoder.ProcessFrame(frame, frame_size, millis());
  if (!event.valid_frame || !event.operational_mode) {
    return;
  }

  tN2kMsg msg;
  sid = static_cast<uint8_t>((sid + 1) & 0xFF);

  if (event.depth_ready) {
    decoder.BuildWaterDepthMessage(msg, sid);
    NMEA2000.SendMsg(msg);
  }

  if (event.speed_ready) {
    decoder.BuildBoatSpeedMessage(msg, sid);
    NMEA2000.SendMsg(msg);
  }

  if (event.distance_log_ready) {
    // Use your own time source in HALMET.
    const double days_since_1970 = 0;
    const double seconds_since_midnight = 0;
    decoder.BuildDistanceLogMessage(msg, days_since_1970, seconds_since_midnight);
    NMEA2000.SendMsg(msg);
  }

  if (event.persist_mask != hamlet::clipperduet::kPersistNone) {
    // Persist settings in HALMET preferences store.
  }
}
