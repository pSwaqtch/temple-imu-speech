#pragma once
// Minimal ICM-20948 driver: accel/gyro through the hardware FIFO, over SPI or I2C.
// Not thread-safe; owned by the sensor task.
#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_err.h"

struct ImuConfig {
  uint8_t  accel_fs;    // 0..3
  uint8_t  gyro_fs;     // 0..3
  uint8_t  accel_dlpf;  // 0..7
  uint8_t  gyro_dlpf;   // 0..7
  uint16_t accel_div;   // 0..4095, ODR = 1125/(1+div)
  uint8_t  gyro_div;    // ODR = 1100/(1+div)
  bool     fifo_gyro;   // include gyro in FIFO frames
};

class ICM20948 {
 public:
  static constexpr int kMaxFramesPerRead = 64;

  // Adds the device to an SPI bus that is already initialized.
  esp_err_t attachSPI(spi_host_device_t host, int cs_pin, int hz);
  // Probes 0x69 then 0x68 on an existing I2C master bus. Safe to call repeatedly.
  esp_err_t attachI2C(i2c_master_bus_handle_t bus, uint32_t hz);

  // Reset, configure and start the FIFO. Call again to recover from faults.
  bool configure(const ImuConfig& cfg);
  bool alive();  // WHO_AM_I matches

  // Drains up to max_frames whole frames from the FIFO into out (int16, host order).
  // Returns frames read (>= 0), or -1 on a bus/sensor fault (caller should re-configure).
  // *overflowed is set if the FIFO overflowed since the last call; the FIFO is then
  // reset and its contents discarded.
  int readFrames(int16_t* out, int max_frames, bool* overflowed);

  uint8_t frameWords() const { return cfg_.fifo_gyro ? 6 : 3; }
  float accelRateHz() const { return 1125.0f / (1 + cfg_.accel_div); }
  const ImuConfig& config() const { return cfg_; }

 private:
  bool readRegs(uint8_t reg, uint8_t* buf, size_t n);
  bool readReg(uint8_t reg, uint8_t* v) { return readRegs(reg, v, 1); }
  bool writeReg(uint8_t reg, uint8_t v);
  bool selectBank(uint8_t bank);
  bool resetFifo();

  spi_device_handle_t spi_ = nullptr;
  i2c_master_dev_handle_t i2c_ = nullptr;

  uint8_t bank_ = 0xFF;  // unknown
  uint8_t user_ctrl_ = 0;
  uint8_t fifo_en2_ = 0;
  ImuConfig cfg_{};
  alignas(4) uint8_t raw_[kMaxFramesPerRead * 12];
};
