# HAMLET-ClipperDuet

Reusable decoder/state library for NASA Clipper Duet display traffic (HT1621 memory frames), with helper methods to build NMEA2000 messages.

This library is intended for integration in projects that already own NMEA2000 transport setup, for example HALMET.

## Wiring for HAMLET Integration

The library only needs HT1621 display bus frames. On hardware, that means tapping three lines from the Clipper Duet display controller and connecting power/ground to your HAMLET ESP32 node.

### Clipper Duet Front PCB Tap Points (PIC)

Use these pins on the Clipper Duet PIC controller:

- PIC pin 4: HT1621 DATA
- PIC pin 5: HT1621 WR/CLK
- PIC pin 7: HT1621 CS
- PIC pin 8: GND
- PIC pin 1: +5V (optional power source for your ESP32 board, only if your power design allows it)

### Recommended ESP32 GPIO Mapping (Same as this firmware)

Use this mapping in HAMLET unless you have a reason to choose other GPIOs:

| Clipper signal | ESP32 signal role | ESP32 GPIO |
| -- | -- | -- |
| HT_DATA (from PIC pin 4) | SPI MOSI input to slave | GPIO 13 |
| HT_DATAOUT | SPI MISO output from slave (required by ESP32 SPI slave driver) | GPIO 14 |
| HT_CLK (from PIC pin 5) | SPI SCLK | GPIO 32 |
| HT_CS (from PIC pin 7) | SPI CS | GPIO 33 |
| GND (from PIC pin 8) | GND | GND |

Notes:

- HT_DATAOUT is not a Clipper wire. It is an ESP32 output pin required by ESP32 SPI slave setup.
- Keep all grounds common between Clipper Duet and HAMLET node.
- Keep these SPI wires short and routed away from noisy power switching paths.

### HALMET Side Setup Checklist

1. Configure your SPI slave capture using the GPIO mapping above (or your chosen mapping).
2. Capture HT1621 memory-write frames (36-byte frame buffer in this implementation).
3. Feed each frame into `Decoder::ProcessFrame(frame, size, millis())`.
4. Publish NMEA2000 messages using HALMET's existing NMEA2000 transport when event flags indicate data is ready.
5. Persist settings when `event.persist_mask` is non-zero.

### Optional Persistence Keys (Suggested)

If HALMET persists values, use keys compatible with this project for easier migration:

- `offset`
- `threshold`
- `cal`
- `shallow_alarm`
- `speed_alarm`

## What It Does

- Decodes one SPI frame from the Clipper Duet HT1621 bus.
- Optionally captures SPI frames on ESP32 via `Esp32SpiCapture`.
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
2. Either:
  - capture frames with the built-in `Esp32SpiCapture` helper, or
  - use your existing HALMET SPI capture path.
3. Feed each captured HT1621 memory frame into `Decoder::ProcessFrame()`.
4. Use event flags to send NMEA2000 messages with your own `NMEA2000.SendMsg(...)`.
5. Persist changed settings when `event.persist_mask` is non-zero.

## Minimal Example

See [examples/basic_n2k_integration/basic_n2k_integration.ino](examples/basic_n2k_integration/basic_n2k_integration.ino).

## API Summary

- `hamlet::clipperduet::Decoder`
- `hamlet::clipperduet::Config`
- `hamlet::clipperduet::Data`
- `hamlet::clipperduet::Event`
- `hamlet::clipperduet::PersistMask`
- `hamlet::clipperduet::Esp32SpiCapture` (ESP32 only, when `ESP32SPISlave` is available)

## Notes

- Frames must be HT1621 memory-write payloads.
- `kFrameBufferSize` is 36 bytes.
- NMEA2000 transport setup remains in the host project.
