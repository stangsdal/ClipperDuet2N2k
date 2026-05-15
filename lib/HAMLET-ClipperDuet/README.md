# HAMLET-ClipperDuet

Reusable decoder/state library for NASA Clipper Duet display traffic (HT1621 memory frames), with helper methods to build NMEA2000 messages.

This library is intended for integration in projects that already own NMEA2000 transport setup, for example HALMET.

## What It Does

- Decodes one SPI frame from the Clipper Duet HT1621 bus.
- Converts display content into normalized SI values:
  - depth in meters
  - speed in m/s
  - trip and total distance in meters
- Tracks settings shown on the Clipper dialogs:
  - offset
  - threshold
  - calibration
  - shallow alarm
  - speed alarm
- Emits event flags so host code decides when to publish or persist.
- Provides NMEA2000 payload builders for:
  - PGN 128267 Water Depth
  - PGN 128259 Boat Speed
  - PGN 128275 Distance Log

## Integration Pattern (HALMET)

1. Initialize your own NMEA2000 stack in HALMET (existing code).
2. Feed each captured HT1621 memory frame into `Decoder::ProcessFrame()`.
3. Use event flags to send NMEA2000 messages with your own `NMEA2000.SendMsg(...)`.
4. Persist changed settings when `event.persist_mask` is non-zero.

## Minimal Example

See [examples/basic_n2k_integration/basic_n2k_integration.ino](examples/basic_n2k_integration/basic_n2k_integration.ino).

## API Summary

- `hamlet::clipperduet::Decoder`
- `hamlet::clipperduet::Config`
- `hamlet::clipperduet::Data`
- `hamlet::clipperduet::Event`
- `hamlet::clipperduet::PersistMask`

## Notes

- Frames must be HT1621 memory-write payloads.
- `kFrameBufferSize` is 36 bytes.
- The library does not own SPI capture or NMEA2000 transport setup.
