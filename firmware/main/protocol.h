#pragma once
// UDP wire format. Mirrored in host/timu/protocol.py -- keep both in sync and bump
// PROTOCOL_VERSION on any change.
//
// PC -> device (to UDP_PORT):  "SUB1" subscribe / keepalive (send ~1/s), "UNSB" unsubscribe.
// Device -> PC (to the sender of SUB1): one data packet = PacketHeader + n_samples frames.
// Each frame is int16 little-endian raw counts: ax ay az [gx gy gz].
//
// Loss accounting:
//   * first_sample counts every sample read from the sensor since boot. Within one
//     segment, a jump in first_sample is an exact count of lost samples (UDP loss or
//     queue_drops), so the PC can fill the gap precisely.
//   * segment increments when samples were lost on the sensor side (FIFO overflow,
//     sensor fault/re-init). The gap size at a segment boundary is unknown; use t_us.
//   * boot_id changes when the device reboots (first_sample restarts at 0).
#include <stdint.h>

#define PROTOCOL_MAGIC   0x554D4954u  // "TIMU" as little-endian bytes
#define PROTOCOL_VERSION 1

#define LAYOUT_ACCEL 0x01
#define LAYOUT_GYRO  0x02

struct __attribute__((packed)) PacketHeader {
  uint32_t magic;
  uint8_t  version;
  uint8_t  layout;          // LAYOUT_* bits
  uint8_t  accel_fs;        // ACCEL_FS_SEL code (0..3)
  uint8_t  gyro_fs;         // GYRO_FS_SEL code (0..3)
  uint16_t n_samples;
  uint16_t segment;
  uint32_t seq;             // packet counter since boot
  uint64_t first_sample;    // sample index of frame 0 in this packet
  uint64_t t_us;            // device time (esp_timer) of the FIFO read that delivered the last frame
  float    nominal_rate_hz; // configured ODR; true rate should be measured from first_sample vs t_us
  uint32_t boot_id;         // random per boot
  uint32_t fifo_overflows;  // cumulative
  uint32_t queue_drops;     // cumulative samples dropped because the TX queue was full
};
static_assert(sizeof(PacketHeader) == 48, "PacketHeader layout changed");
