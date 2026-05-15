#pragma once

#include <stddef.h>
#include <stdint.h>

class tN2kMsg;

namespace hamlet {
namespace clipperduet {

constexpr size_t kFrameBufferSize = 36;

// Persist bits indicate which settings changed while in Clipper settings dialogs.
enum PersistMask : uint32_t {
  kPersistNone = 0,
  kPersistOffset = 1u << 0,
  kPersistThreshold = 1u << 1,
  kPersistCalibration = 1u << 2,
  kPersistShallowAlarm = 1u << 3,
  kPersistSpeedAlarm = 1u << 4,
};

struct Config {
  double safe_offset_m = 3.0;
  uint32_t distance_timeout_ms = 60000;
};

struct Data {
  double speed_mps;
  double depth_m;
  double shallow_alarm_m;
  double speed_alarm_mps;
  double total_m;
  double trip_m;

  double offset_m;
  double threshold_m;
  double calibration_percent;

  uint32_t last_depth_ms;
  uint32_t last_speed_ms;
  uint32_t last_total_ms;
  uint32_t last_trip_ms;
};

struct Event {
  bool valid_frame = false;
  bool operational_mode = false;

  bool depth_ready = false;
  bool speed_ready = false;
  bool distance_log_ready = false;

  bool depth_updated = false;
  bool speed_updated = false;
  bool trip_updated = false;
  bool total_updated = false;

  uint32_t persist_mask = kPersistNone;
};

class Decoder {
 public:
  explicit Decoder(const Config& config = Config());

  // Parse one HT1621 memory-write frame captured from the SPI bus.
  Event ProcessFrame(const uint8_t* frame, size_t frame_size, uint32_t now_ms);

  const Data& data() const { return data_; }

  // Helpers to build NMEA2000 payloads using an already configured N2K stack.
  bool BuildWaterDepthMessage(tN2kMsg& msg, uint8_t sid) const;
  bool BuildBoatSpeedMessage(tN2kMsg& msg, uint8_t sid) const;
  bool BuildDistanceLogMessage(tN2kMsg& msg, double days_since_1970,
                               double seconds_since_midnight) const;

 private:
  struct LCD {
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

  Config config_;
  Data data_{};
  LCD lcd_{};

  uint8_t raw_[kFrameBufferSize]{};
  uint32_t pending_persist_ = kPersistNone;

  void DecodeLCD();
  void ProcessOperational(Event& event, uint32_t now_ms);
  void ProcessSettings();

  double RowAAsDouble() const;
  double RowBAsDouble() const;
};

}  // namespace clipperduet
}  // namespace hamlet
