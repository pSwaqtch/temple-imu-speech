// Temple IMU streamer: ICM-20948 FIFO -> UDP.
//
//   core 1  sensorTask  drains the FIFO every 2 ms, packs SAMPLES_PER_PACKET frames
//                       into a packet, pushes it on g_txq (never blocks on the network)
//   core 0  netTask     handles SUB1/UNSB from the PC, sends queued packets to the subscriber
//   app_main            prints a status line every 2 s
#include <atomic>
#include <cerrno>
#include <cinttypes>
#include <cstring>

#include "config.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "icm20948.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include "protocol.h"
#include "secrets.h"

static const char* TAG = "timu";

static constexpr ImuConfig kImuConfig = {
    IMU_ACCEL_FS, IMU_GYRO_FS, IMU_ACCEL_DLPF, IMU_GYRO_DLPF,
    IMU_ACCEL_DIV, IMU_GYRO_DIV, IMU_FIFO_GYRO != 0,
};

struct TxSlot {
  uint16_t len;
  alignas(4) uint8_t data[sizeof(PacketHeader) + SAMPLES_PER_PACKET * 6 * sizeof(int16_t)];
};

static ICM20948 g_imu;
static QueueHandle_t g_txq;
static uint32_t g_boot_id;

// Counters shared with the status printer.
static std::atomic<uint32_t> g_samples{0}, g_overflows{0}, g_queue_drops{0}, g_faults{0};
static std::atomic<uint32_t> g_tx_ok{0}, g_tx_fail{0};
static std::atomic<bool> g_imu_ready{false}, g_wifi_up{false};
static std::atomic<uint32_t> g_ip{0}, g_ip_gen{0}, g_sub_ip{0};
static std::atomic<uint16_t> g_sub_port{0};

// ---------------------------------------------------------------------------
// Sensor
// ---------------------------------------------------------------------------

static bool imuAttach() {
#if IMU_BUS == IMU_BUS_SPI
  static bool attached = false;
  if (attached) return true;
  spi_bus_config_t bus = {};
  bus.mosi_io_num = PIN_SPI_MOSI;
  bus.miso_io_num = PIN_SPI_MISO;
  bus.sclk_io_num = PIN_SPI_SCK;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = ICM20948::kMaxFramesPerRead * 12 + 8;
  ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO));
  attached = g_imu.attachSPI(SPI3_HOST, PIN_SPI_CS, IMU_SPI_HZ) == ESP_OK;
  return attached;
#else
  static i2c_master_bus_handle_t bus = nullptr;
  if (!bus) {
    i2c_master_bus_config_t cfg = {};
    cfg.i2c_port = I2C_NUM_0;
    cfg.sda_io_num = (gpio_num_t)PIN_I2C_SDA;
    cfg.scl_io_num = (gpio_num_t)PIN_I2C_SCL;
    cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt = 7;
    cfg.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus));
  }
  return g_imu.attachI2C(bus, IMU_I2C_HZ) == ESP_OK;
#endif
}

struct PacketBuilder {
  TxSlot slot;
  int fill = 0;
  uint16_t segment = 0;
  uint32_t seq = 0;
  uint64_t next_sample = 0;
  int64_t last_read_us = 0;

  int16_t* payload() { return reinterpret_cast<int16_t*>(slot.data + sizeof(PacketHeader)); }

  // Sends whatever is buffered (a partial packet is fine) to the TX queue.
  void flush() {
    if (fill == 0) return;
    const ImuConfig& c = g_imu.config();
    auto* h = reinterpret_cast<PacketHeader*>(slot.data);
    h->magic = PROTOCOL_MAGIC;
    h->version = PROTOCOL_VERSION;
    h->layout = LAYOUT_ACCEL | (c.fifo_gyro ? LAYOUT_GYRO : 0);
    h->accel_fs = c.accel_fs;
    h->gyro_fs = c.gyro_fs;
    h->n_samples = fill;
    h->segment = segment;
    h->seq = seq++;
    h->first_sample = next_sample;
    h->t_us = last_read_us;
    h->nominal_rate_hz = g_imu.accelRateHz();
    h->boot_id = g_boot_id;
    h->fifo_overflows = g_overflows.load();
    h->queue_drops = g_queue_drops.load();
    slot.len = sizeof(PacketHeader) + fill * g_imu.frameWords() * sizeof(int16_t);
    next_sample += fill;
    if (xQueueSend(g_txq, &slot, 0) != pdTRUE) g_queue_drops += fill;
    fill = 0;
  }
};

static void sensorTask(void*) {
  static PacketBuilder pb;  // static: keep the packet buffer off the task stack
  bool ready = false;
  int failed_attempts = 0;
  int64_t last_data_us = 0, last_check_us = 0;
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    if (!ready) {
      ready = imuAttach() && g_imu.configure(kImuConfig);
      g_imu_ready = ready;
      if (!ready) {
        if (failed_attempts++ % 10 == 0) ESP_LOGW(TAG, "ICM-20948 not responding, check wiring; retrying");
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
      ESP_LOGI(TAG, "ICM-20948 ready: %.1f Hz, %s", g_imu.accelRateHz(),
               kImuConfig.fifo_gyro ? "accel+gyro" : "accel only");
      failed_attempts = 0;
      pb.segment++;  // samples may have been missed while the sensor was down
      last_data_us = last_check_us = esp_timer_get_time();
      wake = xTaskGetTickCount();
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(SENSOR_POLL_MS));
    const int64_t now = esp_timer_get_time();
    const int words = g_imu.frameWords();
    bool fault = false;

    if (now - last_check_us >= 1000000) {
      last_check_us = now;
      fault = !g_imu.alive();
    }

    while (!fault) {
      bool overflowed = false;
      const int space = SAMPLES_PER_PACKET - pb.fill;
      const int n = g_imu.readFrames(pb.payload() + pb.fill * words, space, &overflowed);
      if (n < 0) {
        fault = true;
        break;
      }
      if (overflowed) {
        g_overflows++;
        pb.flush();
        pb.segment++;
        break;
      }
      if (n == 0) break;
      pb.fill += n;
      g_samples += n;
      last_data_us = pb.last_read_us = esp_timer_get_time();
      if (pb.fill == SAMPLES_PER_PACKET) pb.flush();
      if (n < space) break;  // FIFO drained
    }

    if (!fault && now - last_data_us > SENSOR_STALL_MS * 1000LL) fault = true;
    if (fault) {
      g_faults++;
      ESP_LOGW(TAG, "sensor fault, re-initializing");
      pb.flush();
      ready = false;
      g_imu_ready = false;
    }
  }
}

// ---------------------------------------------------------------------------
// WiFi + UDP
// ---------------------------------------------------------------------------

static void onNetEvent(void*, esp_event_base_t base, int32_t id, void* data) {
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    g_wifi_up = false;
    auto* d = static_cast<wifi_event_sta_disconnected_t*>(data);
    ESP_LOGW(TAG, "wifi disconnected (reason %d), reconnecting", d->reason);
    esp_wifi_connect();
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    auto* e = static_cast<ip_event_got_ip_t*>(data);
    g_ip = e->ip_info.ip.addr;
    g_ip_gen++;
    g_wifi_up = true;
    ESP_LOGI(TAG, "wifi connected, ip " IPSTR, IP2STR(&e->ip_info.ip));
  }
}

static void wifiInit() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_t* sta = esp_netif_create_default_wifi_sta();
  esp_netif_set_hostname(sta, DEVICE_HOSTNAME);

  wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&init));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, onNetEvent, nullptr));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, onNetEvent, nullptr));

  wifi_config_t wc = {};
  strncpy(reinterpret_cast<char*>(wc.sta.ssid), WIFI_SSID, sizeof(wc.sta.ssid));
  strncpy(reinterpret_cast<char*>(wc.sta.password), WIFI_PASS, sizeof(wc.sta.password));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
  ESP_ERROR_CHECK(esp_wifi_start());
  // Modem sleep adds 100+ ms latency spikes and packet loss; this device is always streaming.
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
}

static int openSocket() {
  int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (s < 0) return -1;
  sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(UDP_PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    close(s);
    return -1;
  }
  return s;
}

static void sendPacket(int sock, const sockaddr_in& to, const TxSlot& s) {
  for (int attempt = 0; attempt < 3; attempt++) {
    if (sendto(sock, s.data, s.len, 0, reinterpret_cast<const sockaddr*>(&to), sizeof(to)) == s.len) {
      g_tx_ok++;
      return;
    }
    if (errno != ENOMEM && errno != EAGAIN) break;  // only WiFi TX buffer pressure is worth retrying
    vTaskDelay(1);
  }
  g_tx_fail++;
}

static void netTask(void*) {
  static TxSlot slot;
  int sock = -1;
  uint32_t sock_gen = 0;
  sockaddr_in sub = {};
  bool have_sub = false;
  int64_t sub_seen_us = 0;

  for (;;) {
    if (!g_wifi_up) {
      if (sock >= 0) {
        close(sock);
        sock = -1;
      }
      have_sub = false;
      g_sub_port = 0;
      while (xQueueReceive(g_txq, &slot, 0) == pdTRUE) {}
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    if (sock < 0 || sock_gen != g_ip_gen) {
      if (sock >= 0) close(sock);
      sock = openSocket();
      sock_gen = g_ip_gen;
      have_sub = false;
      if (sock < 0) {
        ESP_LOGE(TAG, "udp socket failed (errno %d)", errno);
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
    }

    // Control messages from the PC.
    const int64_t now = esp_timer_get_time();
    char buf[16];
    sockaddr_in from = {};
    socklen_t from_len = sizeof(from);
    int r;
    while ((r = recvfrom(sock, buf, sizeof(buf), MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&from), &from_len)) > 0) {
      if (r >= 4 && memcmp(buf, "SUB1", 4) == 0) {
        if (!have_sub || from.sin_addr.s_addr != sub.sin_addr.s_addr || from.sin_port != sub.sin_port) {
          ESP_LOGI(TAG, "subscriber %s:%u", inet_ntoa(from.sin_addr), ntohs(from.sin_port));
        }
        sub = from;
        have_sub = true;
        sub_seen_us = now;
      } else if (r >= 4 && memcmp(buf, "UNSB", 4) == 0 && have_sub) {
        ESP_LOGI(TAG, "subscriber left");
        have_sub = false;
      }
      from_len = sizeof(from);
    }
    if (have_sub && now - sub_seen_us > SUBSCRIBER_TIMEOUT_MS * 1000LL) {
      ESP_LOGW(TAG, "subscriber timed out");
      have_sub = false;
    }
    g_sub_ip = sub.sin_addr.s_addr;
    g_sub_port = have_sub ? ntohs(sub.sin_port) : 0;

    // Data. With no subscriber the queue is drained and discarded.
    if (xQueueReceive(g_txq, &slot, pdMS_TO_TICKS(5)) == pdTRUE) {
      do {
        if (have_sub) sendPacket(sock, sub, slot);
      } while (xQueueReceive(g_txq, &slot, 0) == pdTRUE);
    }
  }
}

// ---------------------------------------------------------------------------

extern "C" void app_main() {
  g_boot_id = esp_random();
  ESP_LOGI(TAG, "temple-imu boot_id=%08" PRIx32 " reset_reason=%d", g_boot_id, (int)esp_reset_reason());

  g_txq = xQueueCreate(TX_QUEUE_PACKETS, sizeof(TxSlot));
  configASSERT(g_txq);
  wifiInit();
  xTaskCreatePinnedToCore(sensorTask, "imu", 4096, nullptr, 10, nullptr, 1);
  xTaskCreatePinnedToCore(netTask, "net", 4096, nullptr, 5, nullptr, 0);

  uint32_t prev_samples = 0, prev_tx = 0;
  int64_t prev_us = esp_timer_get_time();
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    const int64_t now = esp_timer_get_time();
    const float dt = (now - prev_us) / 1e6f;
    const uint32_t samples = g_samples, tx = g_tx_ok;
    int rssi = 0;
    wifi_ap_record_t ap;
    if (g_wifi_up && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi = ap.rssi;
    esp_ip4_addr_t ip = {g_ip.load()}, sub_ip = {g_sub_ip.load()};

    ESP_LOGI(TAG,
             "imu=%s %.1f Hz ovf=%" PRIu32 " qdrop=%" PRIu32 " faults=%" PRIu32
             " | wifi=%s ip=" IPSTR " rssi=%d | sub=" IPSTR ":%u tx=%.1f/s fail=%" PRIu32,
             g_imu_ready ? "ok" : "DOWN", (samples - prev_samples) / dt, g_overflows.load(),
             g_queue_drops.load(), g_faults.load(), g_wifi_up ? "up" : "DOWN", IP2STR(&ip), rssi,
             IP2STR(&sub_ip), g_sub_port.load(), (tx - prev_tx) / dt, g_tx_fail.load());
    prev_samples = samples;
    prev_tx = tx;
    prev_us = now;
  }
}
