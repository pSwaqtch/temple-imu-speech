#include "icm20948.h"

#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
// Register addresses (ICM-20948 datasheet DS-000189).
constexpr uint8_t REG_BANK_SEL = 0x7F;
// Bank 0
constexpr uint8_t B0_WHO_AM_I     = 0x00;
constexpr uint8_t B0_USER_CTRL    = 0x03;
constexpr uint8_t B0_LP_CONFIG    = 0x05;
constexpr uint8_t B0_PWR_MGMT_1   = 0x06;
constexpr uint8_t B0_PWR_MGMT_2   = 0x07;
constexpr uint8_t B0_INT_ENABLE_2 = 0x12;
constexpr uint8_t B0_INT_STATUS_2 = 0x1B;
constexpr uint8_t B0_FIFO_EN_1    = 0x66;
constexpr uint8_t B0_FIFO_EN_2    = 0x67;
constexpr uint8_t B0_FIFO_RST     = 0x68;
constexpr uint8_t B0_FIFO_MODE    = 0x69;
constexpr uint8_t B0_FIFO_COUNTH  = 0x70;
constexpr uint8_t B0_FIFO_R_W     = 0x72;
// Bank 2
constexpr uint8_t B2_GYRO_SMPLRT_DIV    = 0x00;
constexpr uint8_t B2_GYRO_CONFIG_1      = 0x01;
constexpr uint8_t B2_ODR_ALIGN_EN       = 0x09;
constexpr uint8_t B2_ACCEL_SMPLRT_DIV_1 = 0x10;
constexpr uint8_t B2_ACCEL_SMPLRT_DIV_2 = 0x11;
constexpr uint8_t B2_ACCEL_CONFIG       = 0x14;

constexpr uint8_t WHO_AM_I_VALUE    = 0xEA;
constexpr uint8_t USER_CTRL_FIFO_EN = 0x40;
constexpr uint8_t USER_CTRL_I2C_DIS = 0x10;
constexpr uint8_t FIFO_EN2_ACCEL    = 0x10;
constexpr uint8_t FIFO_EN2_GYRO_XYZ = 0x0E;

// FIFO_COUNT is 13 bits; a floating SPI MISO reads 0x1FFF. Anything above the real
// FIFO size is a bus fault.
constexpr uint16_t FIFO_COUNT_SANE_MAX = 4096;
constexpr int I2C_TIMEOUT_MS = 20;

void sleepMs(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }
}  // namespace

esp_err_t ICM20948::attachSPI(spi_host_device_t host, int cs_pin, int hz) {
  spi_device_interface_config_t dev = {};
  dev.address_bits = 8;  // register address (+ read bit) goes in the address phase
  dev.mode = 0;
  dev.clock_speed_hz = hz;
  dev.spics_io_num = cs_pin;
  dev.queue_size = 1;
  return spi_bus_add_device(host, &dev, &spi_);
}

esp_err_t ICM20948::attachI2C(i2c_master_bus_handle_t bus, uint32_t hz) {
  if (i2c_) {
    i2c_master_bus_rm_device(i2c_);
    i2c_ = nullptr;
  }
  static constexpr uint16_t kAddrs[] = {0x69, 0x68};
  for (uint16_t addr : kAddrs) {
    if (i2c_master_probe(bus, addr, I2C_TIMEOUT_MS) != ESP_OK) continue;
    i2c_device_config_t dev = {};
    dev.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev.device_address = addr;
    dev.scl_speed_hz = hz;
    if (i2c_master_bus_add_device(bus, &dev, &i2c_) != ESP_OK) continue;
    bank_ = 0xFF;
    if (alive()) return ESP_OK;
    i2c_master_bus_rm_device(i2c_);
    i2c_ = nullptr;
  }
  return ESP_ERR_NOT_FOUND;
}

bool ICM20948::readRegs(uint8_t reg, uint8_t* buf, size_t n) {
  if (spi_) {
    spi_transaction_t t = {};
    t.addr = reg | 0x80;
    t.length = n * 8;
    t.rxlength = n * 8;
    t.rx_buffer = buf;
    return spi_device_polling_transmit(spi_, &t) == ESP_OK;
  }
  if (i2c_) return i2c_master_transmit_receive(i2c_, &reg, 1, buf, n, I2C_TIMEOUT_MS) == ESP_OK;
  return false;
}

bool ICM20948::writeReg(uint8_t reg, uint8_t v) {
  if (spi_) {
    spi_transaction_t t = {};
    t.flags = SPI_TRANS_USE_TXDATA;
    t.addr = reg & 0x7F;
    t.length = 8;
    t.tx_data[0] = v;
    return spi_device_polling_transmit(spi_, &t) == ESP_OK;
  }
  if (i2c_) {
    const uint8_t d[2] = {reg, v};
    return i2c_master_transmit(i2c_, d, 2, I2C_TIMEOUT_MS) == ESP_OK;
  }
  return false;
}

bool ICM20948::selectBank(uint8_t bank) {
  if (bank == bank_) return true;
  if (!writeReg(REG_BANK_SEL, bank << 4)) {
    bank_ = 0xFF;
    return false;
  }
  bank_ = bank;
  return true;
}

bool ICM20948::alive() {
  uint8_t who = 0;
  return selectBank(0) && readReg(B0_WHO_AM_I, &who) && who == WHO_AM_I_VALUE;
}

bool ICM20948::configure(const ImuConfig& c) {
  cfg_ = c;
  bank_ = 0xFF;
  bool ok = selectBank(0) && writeReg(B0_PWR_MGMT_1, 0x80);  // device reset
  sleepMs(100);
  bank_ = 0xFF;  // reset returns to bank 0, but don't trust the cache
  user_ctrl_ = spi_ ? USER_CTRL_I2C_DIS : 0;  // lock interface to SPI so I2C noise can't hijack it
  ok = ok && selectBank(0) && writeReg(B0_USER_CTRL, user_ctrl_);
  ok = ok && writeReg(B0_PWR_MGMT_1, 0x01);  // wake, auto-select best clock
  sleepMs(50);
  if (!ok || !alive()) return false;

  ok = writeReg(B0_PWR_MGMT_2, c.fifo_gyro ? 0x00 : 0x07);  // accel on, gyro on/off
  ok = ok && writeReg(B0_LP_CONFIG, 0x00);                   // continuous (low-noise) mode

  const uint8_t accel_cfg = ((c.accel_dlpf & 7) << 3) | ((c.accel_fs & 3) << 1) | 0x01;
  const uint8_t gyro_cfg = ((c.gyro_dlpf & 7) << 3) | ((c.gyro_fs & 3) << 1) | 0x01;
  ok = ok && selectBank(2);
  ok = ok && writeReg(B2_GYRO_SMPLRT_DIV, c.gyro_div);
  ok = ok && writeReg(B2_GYRO_CONFIG_1, gyro_cfg);
  ok = ok && writeReg(B2_ACCEL_SMPLRT_DIV_1, (c.accel_div >> 8) & 0x0F);
  ok = ok && writeReg(B2_ACCEL_SMPLRT_DIV_2, c.accel_div & 0xFF);
  ok = ok && writeReg(B2_ACCEL_CONFIG, accel_cfg);
  ok = ok && writeReg(B2_ODR_ALIGN_EN, 0x01);
  // Read back: SPI has no ACK, so this is how a broken bus gets caught.
  uint8_t rb = 0;
  ok = ok && readReg(B2_ACCEL_CONFIG, &rb) && rb == accel_cfg;
  ok = ok && selectBank(0);
  if (!ok) return false;
  sleepMs(50);  // gyro start-up time

  fifo_en2_ = FIFO_EN2_ACCEL | (c.fifo_gyro ? FIFO_EN2_GYRO_XYZ : 0);
  ok = writeReg(B0_FIFO_EN_1, 0x00);
  ok = ok && writeReg(B0_FIFO_MODE, 0x00);     // stream mode
  ok = ok && writeReg(B0_INT_ENABLE_2, 0x01);  // FIFO overflow status
  ok = ok && writeReg(B0_USER_CTRL, user_ctrl_ | USER_CTRL_FIFO_EN);
  return ok && resetFifo();
}

bool ICM20948::resetFifo() {
  uint8_t st;
  bool ok = writeReg(B0_FIFO_EN_2, 0x00);
  ok = ok && writeReg(B0_FIFO_RST, 0x1F);
  ok = ok && writeReg(B0_FIFO_RST, 0x00);
  ok = ok && readReg(B0_INT_STATUS_2, &st);  // clears the overflow flag
  return ok && writeReg(B0_FIFO_EN_2, fifo_en2_);
}

int ICM20948::readFrames(int16_t* out, int max_frames, bool* overflowed) {
  *overflowed = false;
  uint8_t st;
  if (!readReg(B0_INT_STATUS_2, &st)) return -1;
  if (st & 0x1F) {
    *overflowed = true;
    return resetFifo() ? 0 : -1;
  }

  uint8_t c[2];
  if (!readRegs(B0_FIFO_COUNTH, c, 2)) return -1;
  const uint16_t count = ((c[0] & 0x1F) << 8) | c[1];
  if (count > FIFO_COUNT_SANE_MAX) return -1;

  const uint8_t words = frameWords();
  const uint8_t frame_bytes = words * 2;
  int frames = count / frame_bytes;
  if (frames > max_frames) frames = max_frames;
  if (frames > kMaxFramesPerRead) frames = kMaxFramesPerRead;
  if (frames <= 0) return 0;

  if (!readRegs(B0_FIFO_R_W, raw_, frames * frame_bytes)) return -1;
  // FIFO data is big-endian, in register order: accel XYZ then gyro XYZ.
  for (int i = 0; i < frames * words; i++) {
    out[i] = (int16_t)((raw_[2 * i] << 8) | raw_[2 * i + 1]);
  }
  return frames;
}
