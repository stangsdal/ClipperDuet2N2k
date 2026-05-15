#include "HAMLET_ClipperDuet.h"

#include <N2kMessages.h>

namespace hamlet {
namespace clipperduet {

namespace {

constexpr char kSevenSegToChar[128] = {
    ' ', '~', '\'', '>', '.', 'i', '1', '7', '_', ':', ';', ' ', ',', 'J', 'J', ')',
    '.', 'I', 'X', '?', 'X', ' ', ' ', '@', ' ', 'i', ' ', 'Z', 'U', 'U', 'J', 'J',
    '\'', '<', '"', ' ', '%', ' ', ' ', '7', ' ', ' ', 'V', '!', ' ', 'S', ' ', ' ',
    '1', ' ', ' ', 's', ' ', ' ', '#', 'N', 'L', '(', ' ', ' ', ' ', 'G', 'U', '0',
    '-', ' ', ' ', ' ', ' ', ' ', '+', ' ', '=', '*', ' ', '?', ' ', 'A', ' ', '3',
    'R', ' ', '/', '?', 'N', 'M', ' ', ' ', 'C', 'A', ' ', '2', 'O', 'O', 'D', 'A',
    ' ', ' ', ' ', '~', '\\', ' ', '4', 'Q', ' ', ' ', 'W', '!', ' ', '5', 'Y', '9',
    '+', 'F', 'V', 'P', 'H', 'K', 'X', 'A', 'T', 'E', ' ', ' ', 'B', '6', ' ', '8'};

inline bool IsMemoryWriteFrame(const uint8_t* frame, size_t size) {
  return size > 2 && size < kFrameBufferSize && ((frame[0] & 0b11100000) == 0b10100000);
}

inline uint8_t SegData(uint8_t seg, uint8_t com, const uint8_t* buf) {
  const uint8_t shift_by = 3 + 6;
  return ((buf[(seg * 4 + com + shift_by) / 8] >>
           (7 - ((seg * 4 + com + shift_by) % 8))) &
          1);
}

inline uint8_t MkDigit0(const uint8_t* b) {
  return (SegData(13, 1, b) << 6) | (SegData(14, 0, b) << 5) |
         (SegData(14, 1, b) << 4) | (SegData(15, 1, b) << 3) |
         (SegData(12, 1, b) << 2) | (SegData(12, 0, b) << 1) |
         (SegData(13, 0, b));
}

inline uint8_t MkDigit1(const uint8_t* b) {
  return (SegData(11, 1, b) << 6) | (SegData(15, 0, b) << 5) |
         (SegData(7, 1, b) << 4) | (SegData(7, 0, b) << 3) |
         (SegData(10, 1, b) << 2) | (SegData(10, 0, b) << 1) |
         (SegData(11, 0, b));
}

inline uint8_t MkDigit2(const uint8_t* b) {
  return (SegData(8, 1, b) << 6) | (SegData(9, 0, b) << 5) |
         (SegData(9, 1, b) << 4) | (SegData(6, 1, b) << 3) |
         (SegData(16, 1, b) << 2) | (SegData(16, 0, b) << 1) |
         (SegData(8, 0, b)) | (SegData(6, 0, b) << 7);
}

inline uint8_t MkDigit3(const uint8_t* b) {
  return (SegData(26, 1, b) << 6) | (SegData(25, 0, b) << 5) |
         (SegData(25, 1, b) << 4) | (SegData(28, 1, b) << 3) |
         (SegData(27, 1, b) << 2) | (SegData(27, 0, b) << 1) |
         (SegData(26, 0, b));
}

inline uint8_t MkDigit4(const uint8_t* b) {
  return (SegData(3, 1, b) << 6) | (SegData(4, 1, b) << 5) |
         (SegData(4, 0, b) << 4) | (SegData(3, 0, b) << 3) |
         (SegData(2, 0, b) << 2) | (SegData(2, 1, b) << 1) |
         (SegData(5, 1, b));
}

inline uint8_t MkDigit5(const uint8_t* b) {
  return (SegData(0, 1, b) << 6) | (SegData(1, 1, b) << 5) |
         (SegData(1, 0, b) << 4) | (SegData(0, 0, b) << 3) |
         (SegData(17, 0, b) << 2) | (SegData(17, 1, b) << 1) |
         (SegData(23, 1, b)) | (SegData(23, 0, b) << 7);
}

inline uint8_t MkDigit6(const uint8_t* b) {
  return (SegData(18, 1, b) << 6) | (SegData(22, 1, b) << 5) |
         (SegData(22, 0, b) << 4) | (SegData(18, 0, b) << 3) |
         (SegData(19, 0, b) << 2) | (SegData(19, 1, b) << 1) |
         (SegData(21, 1, b));
}

inline uint8_t MkInfo1(const uint8_t* b) {
  const uint8_t trip = SegData(28, 0, b) << 7;
  const uint8_t total = SegData(29, 0, b) << 6;
  const uint8_t kts = SegData(31, 1, b) << 5;
  const uint8_t mph = SegData(30, 0, b) << 4;
  const uint8_t km = SegData(30, 1, b) << 3;
  const uint8_t ph = SegData(31, 0, b) << 2;
  const uint8_t n = SegData(29, 1, b) << 1;
  const uint8_t miles = SegData(24, 1, b);
  return trip | total | kts | mph | km | ph | n | miles;
}

inline uint8_t MkInfo2(const uint8_t* b) {
  const uint8_t rowa_dot = SegData(6, 0, b) << 5;
  const uint8_t rowb_dot = SegData(23, 0, b) << 4;
  const uint8_t bell = SegData(5, 0, b) << 3;
  const uint8_t line = SegData(24, 0, b) << 2;
  const uint8_t depth_ft = SegData(20, 1, b) << 1;
  const uint8_t depth_m = SegData(20, 0, b);
  return rowa_dot | rowb_dot | bell | line | depth_ft | depth_m;
}

inline uint32_t DigitsToInt(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
                            bool dot) {
  int x = N2kUInt32NA;

  if (d0 >= '0' && d0 <= '9') {
    x = d0 - '0';
    x *= 10;
  }

  if (d1 == ' ') {
    x = 0;
  } else if (d1 >= '0' && d1 <= '9') {
    if (x == N2kUInt32NA) {
      x = d1 - '0';
    } else {
      x += d1 - '0';
    }
    x *= 10;
  } else {
    return N2kUInt32NA;
  }

  if (d2 >= '0' && d2 <= '9') {
    x += d2 - '0';
    x *= 10;
  } else {
    return N2kUInt32NA;
  }

  if (d3 >= '0' && d3 <= '9') {
    x += d3 - '0';
  } else {
    return N2kUInt32NA;
  }

  return dot ? x : x * 10;
}

inline double NmToM(double nm) { return nm * 1852.0; }
inline double KmToM(double km) { return km * 1000.0; }
inline double MilesToM(double mi) { return mi * 1609.0; }

}  // namespace

Decoder::Decoder(const Config& config) : config_(config) {
  data_.speed_mps = N2kDoubleNA;
  data_.depth_m = N2kDoubleNA;
  data_.shallow_alarm_m = 0.0;
  data_.speed_alarm_mps = 0.0;
  data_.total_m = N2kDoubleNA;
  data_.trip_m = N2kDoubleNA;

  data_.offset_m = -config_.safe_offset_m;
  data_.threshold_m = 0.0;
  data_.calibration_percent = 100.0;

  data_.last_depth_ms = 0;
  data_.last_speed_ms = 0;
  data_.last_total_ms = 0;
  data_.last_trip_ms = 0;
}

Event Decoder::ProcessFrame(const uint8_t* frame, size_t frame_size,
                            uint32_t now_ms) {
  Event event;

  if (!frame || !IsMemoryWriteFrame(frame, frame_size)) {
    return event;
  }

  for (size_t i = 0; i < frame_size; i++) {
    raw_[i] = frame[i];
  }

  DecodeLCD();
  event.valid_frame = true;
  event.operational_mode = (lcd_.info2 & (1 << 2)) == (1 << 2);

  if (event.operational_mode) {
    if (pending_persist_ != kPersistNone) {
      event.persist_mask = pending_persist_;
      pending_persist_ = kPersistNone;
    }
    ProcessOperational(event, now_ms);
  } else {
    ProcessSettings();
  }

  return event;
}

void Decoder::DecodeLCD() {
  lcd_.digit0 = kSevenSegToChar[MkDigit0(raw_) & 0x7F];
  lcd_.digit1 = kSevenSegToChar[MkDigit1(raw_) & 0x7F];
  lcd_.digit2 = kSevenSegToChar[MkDigit2(raw_) & 0x7F];
  lcd_.digit3 = kSevenSegToChar[MkDigit3(raw_) & 0x7F];
  lcd_.digit4 = kSevenSegToChar[MkDigit4(raw_) & 0x7F];
  lcd_.digit5 = kSevenSegToChar[MkDigit5(raw_) & 0x7F];
  lcd_.digit6 = kSevenSegToChar[MkDigit6(raw_) & 0x7F];
  lcd_.info1 = MkInfo1(raw_);
  lcd_.info2 = MkInfo2(raw_);
}

double Decoder::RowAAsDouble() const {
  const uint32_t val = DigitsToInt(lcd_.digit0, lcd_.digit1, lcd_.digit2,
                                   lcd_.digit3,
                                   (lcd_.info2 & (1 << 5)) == (1 << 5));
  return (val != N2kUInt32NA) ? (static_cast<double>(val) / 10.0) : N2kDoubleNA;
}

double Decoder::RowBAsDouble() const {
  const uint32_t val = DigitsToInt(' ', lcd_.digit4, lcd_.digit5, lcd_.digit6,
                                   (lcd_.info2 & (1 << 4)) == (1 << 4));
  return (val != N2kUInt32NA) ? (static_cast<double>(val) / 10.0) : N2kDoubleNA;
}

void Decoder::ProcessOperational(Event& event, uint32_t now_ms) {
  const double depth = RowBAsDouble();
  if (depth != N2kDoubleNA) {
    if ((lcd_.info2 & 1) == 1) {
      data_.depth_m = depth;
      data_.last_depth_ms = now_ms;
      event.depth_updated = true;
    } else if ((lcd_.info2 & (1 << 1)) == (1 << 1)) {
      data_.depth_m = depth / 3.281;
      data_.last_depth_ms = now_ms;
      event.depth_updated = true;
    } else {
      data_.depth_m = N2kDoubleNA;
    }
  }
  event.depth_ready = true;

  if (lcd_.info1 < (1 << 6)) {
    const double speed = RowAAsDouble();
    if (speed != N2kDoubleNA) {
      if ((lcd_.info1 & 0b00100000) == 0b00100000) {
        data_.speed_mps = speed * 0.514444444444;
      } else if ((lcd_.info1 & 0b00001100) == 0b00001100) {
        data_.speed_mps = speed / 3.6;
      } else if ((lcd_.info1 & 0b00010000) == 0b00010000) {
        data_.speed_mps = speed / 2.237;
      } else {
        data_.speed_mps = 0.0;
      }
      data_.last_speed_ms = now_ms;
      event.speed_updated = true;
    } else {
      data_.speed_mps = N2kDoubleNA;
    }

    event.speed_ready = true;
    return;
  }

  double distance = RowAAsDouble();
  if (distance != N2kDoubleNA) {
    if ((lcd_.info1 & 0b00000011) == 0b00000011) {
      distance = NmToM(distance);
    } else if ((lcd_.info1 & 0b00001000) == 0b00001000) {
      distance = KmToM(distance);
    } else if ((lcd_.info1 & 0b00000001) == 0b00000001) {
      distance = MilesToM(distance);
    } else {
      distance = N2kDoubleNA;
    }
  }

  if (distance == N2kDoubleNA) {
    return;
  }

  if ((lcd_.info1 & 0b10000000) == 0b10000000) {
    data_.trip_m = distance;
    data_.last_trip_ms = now_ms;
    event.trip_updated = true;
    event.distance_log_ready = true;
    return;
  }

  if ((lcd_.info1 & 0b01000000) == 0b01000000) {
    data_.total_m = distance;
    data_.last_total_ms = now_ms;
    event.total_updated = true;

    if ((now_ms - data_.last_trip_ms) < config_.distance_timeout_ms) {
      event.distance_log_ready = true;
    }
  }
}

void Decoder::ProcessSettings() {
  if (lcd_.digit0 == 'W' && lcd_.digit1 == '(') {
    double offset = RowBAsDouble();
    if (offset != N2kDoubleNA) {
      if ((lcd_.info2 & 1) == 1) {
        offset = -offset;
      } else if ((lcd_.info2 & (1 << 1)) == (1 << 1)) {
        offset = -(offset / 3.281);
      } else {
        offset = -config_.safe_offset_m;
      }
      if (offset != data_.offset_m) {
        data_.offset_m = offset;
        pending_persist_ |= kPersistOffset;
      }
    }
    return;
  }

  if (lcd_.digit0 == 'T' && lcd_.digit1 == '(') {
    double threshold = RowBAsDouble();
    if (threshold != N2kDoubleNA) {
      if ((lcd_.info2 & (1 << 1)) == (1 << 1)) {
        threshold = threshold / 3.281;
      } else if (!((lcd_.info2 & 1) == 1)) {
        threshold = 0.0;
      }
      if (threshold != data_.threshold_m) {
        data_.threshold_m = threshold;
        pending_persist_ |= kPersistThreshold;
      }
    }
    return;
  }

  if (lcd_.digit4 == '(' && ((lcd_.info2 & 0b100011) == 0)) {
    const double calibration = RowAAsDouble();
    if (calibration != N2kDoubleNA && calibration != data_.calibration_percent) {
      data_.calibration_percent = calibration;
      pending_persist_ |= kPersistCalibration;
    }
    return;
  }

  if (lcd_.digit1 == '5' && lcd_.digit2 == 'X') {
    const double shallow_alarm = RowBAsDouble();
    if (shallow_alarm != N2kDoubleNA) {
      if ((lcd_.info2 & 1) == 1) {
        data_.shallow_alarm_m = shallow_alarm;
      } else if ((lcd_.info2 & (1 << 1)) == (1 << 1)) {
        data_.shallow_alarm_m = shallow_alarm / 3.281;
      } else {
        data_.shallow_alarm_m = config_.safe_offset_m + 1.0;
      }
      pending_persist_ |= kPersistShallowAlarm;
    }
    return;
  }

  if (lcd_.digit4 == '5' && lcd_.digit5 == 'P') {
    const double speed_alarm = RowAAsDouble();
    if (speed_alarm != N2kDoubleNA) {
      if ((lcd_.info1 & 0b00100000) == 0b00100000) {
        data_.speed_alarm_mps = speed_alarm * 0.514444444444;
      } else if ((lcd_.info1 & 0b00001100) == 0b00001100) {
        data_.speed_alarm_mps = speed_alarm / 3.6;
      } else if ((lcd_.info1 & 0b00010000) == 0b00010000) {
        data_.speed_alarm_mps = speed_alarm / 2.237;
      }
    } else {
      data_.speed_alarm_mps = 0.0;
    }
    pending_persist_ |= kPersistSpeedAlarm;
  }
}

bool Decoder::BuildWaterDepthMessage(tN2kMsg& msg, uint8_t sid) const {
  SetN2kWaterDepth(msg, sid, data_.depth_m, data_.offset_m, N2kDoubleNA);
  return true;
}

bool Decoder::BuildBoatSpeedMessage(tN2kMsg& msg, uint8_t sid) const {
  SetN2kBoatSpeed(msg, sid, data_.speed_mps, N2kDoubleNA, N2kSWRT_Paddle_wheel);
  return true;
}

bool Decoder::BuildDistanceLogMessage(tN2kMsg& msg, double days_since_1970,
                                      double seconds_since_midnight) const {
  SetN2kDistanceLog(msg, days_since_1970, seconds_since_midnight, data_.total_m,
                    data_.trip_m);
  return true;
}

}  // namespace clipperduet
}  // namespace hamlet
