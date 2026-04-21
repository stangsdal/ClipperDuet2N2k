#include "lcd_decode.h"

#include <N2kMessages.h>

#include <sevenseg2char.h>

#include "clipper_data.h"

uint8_t spi_slave_rx_buf[BUFFER_SIZE]{};

// receive buffer contains 3 command bits followed by 6 address bits, then lcd memory follows
static inline uint8_t segdata(uint8_t seg, uint8_t com, uint8_t* buf) {
  const uint8_t shift_by = 3 + 6;
  return ((buf[(seg * 4 + com + shift_by) / 8] >>
           (7 - ((seg * 4 + com + shift_by) % 8))) &
          1);
}

// Bit order 7..0: (dot) g f e d c b a
static inline uint8_t mkdigit0(uint8_t* buf) {
  return (segdata(13, 1, buf) << 6) | (segdata(14, 0, buf) << 5) |
         (segdata(14, 1, buf) << 4) | (segdata(15, 1, buf) << 3) |
         (segdata(12, 1, buf) << 2) | (segdata(12, 0, buf) << 1) |
         (segdata(13, 0, buf));
}

static inline uint8_t mkdigit1(uint8_t* buf) {
  return (segdata(11, 1, buf) << 6) | (segdata(15, 0, buf) << 5) |
         (segdata(7, 1, buf) << 4) | (segdata(7, 0, buf) << 3) |
         (segdata(10, 1, buf) << 2) | (segdata(10, 0, buf) << 1) |
         (segdata(11, 0, buf));
}

static inline uint8_t mkdigit2(uint8_t* buf) {
  return (segdata(8, 1, buf) << 6) | (segdata(9, 0, buf) << 5) |
         (segdata(9, 1, buf) << 4) | (segdata(6, 1, buf) << 3) |
         (segdata(16, 1, buf) << 2) | (segdata(16, 0, buf) << 1) |
         (segdata(8, 0, buf)) | (segdata(6, 0, buf) << 7);
}

static inline uint8_t mkdigit3(uint8_t* buf) {
  return (segdata(26, 1, buf) << 6) | (segdata(25, 0, buf) << 5) |
         (segdata(25, 1, buf) << 4) | (segdata(28, 1, buf) << 3) |
         (segdata(27, 1, buf) << 2) | (segdata(27, 0, buf) << 1) |
         (segdata(26, 0, buf));
}

static inline uint8_t mkdigit4(uint8_t* buf) {
  return (segdata(3, 1, buf) << 6) | (segdata(4, 1, buf) << 5) |
         (segdata(4, 0, buf) << 4) | (segdata(3, 0, buf) << 3) |
         (segdata(2, 0, buf) << 2) | (segdata(2, 1, buf) << 1) |
         (segdata(5, 1, buf));
}

static inline uint8_t mkdigit5(uint8_t* buf) {
  return (segdata(0, 1, buf) << 6) | (segdata(1, 1, buf) << 5) |
         (segdata(1, 0, buf) << 4) | (segdata(0, 0, buf) << 3) |
         (segdata(17, 0, buf) << 2) | (segdata(17, 1, buf) << 1) |
         (segdata(23, 1, buf)) | (segdata(23, 0, buf) << 7);
}

static inline uint8_t mkdigit6(uint8_t* buf) {
  return (segdata(18, 1, buf) << 6) | (segdata(22, 1, buf) << 5) |
         (segdata(22, 0, buf) << 4) | (segdata(18, 0, buf) << 3) |
         (segdata(19, 0, buf) << 2) | (segdata(19, 1, buf) << 1) |
         (segdata(21, 1, buf));
}

static inline uint8_t i1_trip(uint8_t* buf) { return (segdata(28, 0, buf) << 7); }
static inline uint8_t i1_total(uint8_t* buf) { return (segdata(29, 0, buf) << 6); }

#ifdef OLD_SPI_COMMS
static inline uint8_t i1_kts(uint8_t* buf) { return (buf[16] & 1); }
#else
static inline uint8_t i1_kts(uint8_t* buf) { return (segdata(31, 1, buf) << 5); }
#endif

static inline uint8_t i1_mph(uint8_t* buf) { return (segdata(30, 0, buf) << 4); }
static inline uint8_t i1_km(uint8_t* buf) { return (segdata(30, 1, buf) << 3); }

#ifdef OLD_SPI_COMMS
static inline uint8_t i1_ph(uint8_t* buf) { return ((buf[16] >> 1) & 1); }
#else
static inline uint8_t i1_ph(uint8_t* buf) { return (segdata(31, 0, buf) << 2); }
#endif

static inline uint8_t i1_n(uint8_t* buf) { return (segdata(29, 1, buf) << 1); }
static inline uint8_t i1_miles(uint8_t* buf) { return (segdata(24, 1, buf)); }

static inline uint8_t mkinfo1(uint8_t* buf) {
  return (i1_trip(buf) | i1_total(buf) | i1_kts(buf) | i1_mph(buf) | i1_km(buf) |
          i1_ph(buf) | i1_n(buf) | i1_miles(buf));
}

static inline uint8_t i2_rowadot(uint8_t* buf) { return (segdata(6, 0, buf) << 5); }
static inline uint8_t i2_rowbdot(uint8_t* buf) { return (segdata(23, 0, buf) << 4); }
static inline uint8_t i2_bell(uint8_t* buf) { return (segdata(5, 0, buf) << 3); }
static inline uint8_t i2_line(uint8_t* buf) { return (segdata(24, 0, buf) << 2); }
static inline uint8_t i2_depthft(uint8_t* buf) { return (segdata(20, 1, buf) << 1); }
static inline uint8_t i2_depthm(uint8_t* buf) { return (segdata(20, 0, buf)); }

static inline uint8_t mkinfo2(uint8_t* buf) {
  return (i2_rowadot(buf) | i2_rowbdot(buf) | i2_bell(buf) | i2_line(buf) |
          i2_depthft(buf) | i2_depthm(buf));
}

void buf2clipperlcd() {
  clipperlcd.digit0 = SevenSeg2Char[mkdigit0(spi_slave_rx_buf) & 0x7f];
  clipperlcd.digit1 = SevenSeg2Char[mkdigit1(spi_slave_rx_buf) & 0x7f];
  clipperlcd.digit2 = SevenSeg2Char[mkdigit2(spi_slave_rx_buf) & 0x7f];
  clipperlcd.digit3 = SevenSeg2Char[mkdigit3(spi_slave_rx_buf) & 0x7f];
  clipperlcd.digit4 = SevenSeg2Char[mkdigit4(spi_slave_rx_buf) & 0x7f];
  clipperlcd.digit5 = SevenSeg2Char[mkdigit5(spi_slave_rx_buf) & 0x7f];
  clipperlcd.digit6 = SevenSeg2Char[mkdigit6(spi_slave_rx_buf) & 0x7f];
  clipperlcd.info1 = mkinfo1(spi_slave_rx_buf);
  clipperlcd.info2 = mkinfo2(spi_slave_rx_buf);
}

uint32_t digits2int(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, bool dot) {
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

double rowa2double() {
  const uint32_t val = digits2int(
      clipperlcd.digit0, clipperlcd.digit1, clipperlcd.digit2, clipperlcd.digit3,
      ((clipperlcd.info2 & (1 << 5)) == (1 << 5)));

  if (val != N2kUInt32NA) {
    return static_cast<double>(val) / 10.0;
  }
  return N2kDoubleNA;
}

double rowb2double() {
  const uint32_t val = digits2int(
      ' ', clipperlcd.digit4, clipperlcd.digit5, clipperlcd.digit6,
      ((clipperlcd.info2 & (1 << 4)) == (1 << 4)));

  if (val != N2kUInt32NA) {
    return static_cast<double>(val) / 10.0;
  }
  return N2kDoubleNA;
}

void printBits(void const* ptr, size_t size) {
  unsigned char* b = (unsigned char*)ptr;
  unsigned char byte;

  for (size_t i = 0; i < size; i++) {
    for (int j = 7; j >= 0; j--) {
      byte = (b[i] >> j) & 1;
      printf("%u", byte);
    }
    printf(" ");
  }
}
