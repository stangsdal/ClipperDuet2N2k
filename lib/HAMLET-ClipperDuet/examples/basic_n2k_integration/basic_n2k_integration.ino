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

#if defined(HAMLET_CLIPPERDUET_HAS_SPI_CAPTURE)
hamlet::clipperduet::Esp32SpiCapture capture({
    32,  // HT_CLK
    14,  // HT_DATAOUT (required MISO output)
    13,  // HT_DATA
    33   // HT_CS
});
#endif

void setup() {
  // HALMET project should keep its existing NMEA2000 setup here.
  // Example only:
  // NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 22);
  // NMEA2000.Open();

#if defined(HAMLET_CLIPPERDUET_HAS_SPI_CAPTURE)
  capture.Begin();
#endif
}

void loop() {
  NMEA2000.ParseMessages();

  uint8_t frame[hamlet::clipperduet::kFrameBufferSize] = {0};
  size_t frame_size = 0;

  bool have_frame = false;
#if defined(HAMLET_CLIPPERDUET_HAS_SPI_CAPTURE)
  have_frame = capture.ReadFrame(frame, &frame_size);
#else
  have_frame = ReadClipperFrame(frame, &frame_size);
#endif

  if (!have_frame) {
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
