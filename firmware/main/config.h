#pragma once
// All build-time settings live here. WiFi credentials are in secrets.h.

// ---------------------------------------------------------------------------
// Sensor bus. SPI is the default: it has headroom for future high-rate modes
// (DLPF bypass, ~4.5 kHz accel) that 400 kHz I2C cannot carry.
// ---------------------------------------------------------------------------
#define IMU_BUS_SPI 1
#define IMU_BUS_I2C 2
#define IMU_BUS IMU_BUS_SPI

// SPI3 (VSPI) native IO_MUX pins
#define PIN_SPI_SCK   18  // -> board SCL / SCLK
#define PIN_SPI_MISO  19  // -> board AD0 / SDO
#define PIN_SPI_MOSI  23  // -> board SDA / SDI
#define PIN_SPI_CS     5  // -> board CS / NCS
#define IMU_SPI_HZ  4000000  // chip max is 7 MHz; 4 MHz leaves margin for a cable to the head

// I2C fallback (only used when IMU_BUS == IMU_BUS_I2C). Addresses 0x69 and 0x68 are both probed.
#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22
#define IMU_I2C_HZ  400000

// ---------------------------------------------------------------------------
// Sampling. Speech fundamental is ~85-255 Hz; bone-conducted harmonics go higher,
// so the accelerometer runs at its max DLPF-filtered rate.
//   accel ODR = 1125 / (1 + IMU_ACCEL_DIV) Hz, DLPF 7 -> 473 Hz bandwidth
//   gyro  ODR = 1100 / (1 + IMU_GYRO_DIV)  Hz, DLPF 7 -> 361 Hz bandwidth
// ---------------------------------------------------------------------------
#define IMU_ACCEL_DIV   0
#define IMU_GYRO_DIV    0
#define IMU_ACCEL_DLPF  7
#define IMU_GYRO_DLPF   7
#define IMU_ACCEL_FS    0   // 0:+-2g 1:+-4g 2:+-8g 3:+-16g  (2g = finest resolution)
#define IMU_GYRO_FS     0   // 0:250 1:500 2:1000 3:2000 dps
#define IMU_FIFO_GYRO   1   // 1: frames are accel+gyro (12 B), 0: accel only (6 B)

#define SENSOR_POLL_MS      2    // FIFO drain interval
#define SENSOR_STALL_MS   200    // no data for this long -> sensor treated as faulted, re-init

// ---------------------------------------------------------------------------
// Network
// ---------------------------------------------------------------------------
#define DEVICE_HOSTNAME        "temple-imu"
#define UDP_PORT               5555  // device listens here for SUB1 / UNSB from the PC
#define SAMPLES_PER_PACKET     32    // ~28 ms of data per packet at 1125 Hz
#define TX_QUEUE_PACKETS       96    // ~2.7 s of buffering during WiFi hiccups
#define SUBSCRIBER_TIMEOUT_MS  5000  // stop streaming if the PC stops sending keepalives
