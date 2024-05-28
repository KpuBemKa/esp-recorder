#include <cstring>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "i2s_sampler.hpp"
#include "sd_card.hpp"
#include "wav_reader.hpp"
#include "wav_writer.hpp"
#include "communication.hpp"

#include "config.h"
#include "server.h"

#include "settings.hpp"

extern int32_t currentmax;

#define EXAMPLE_ESP_WIFI_SSID "MARS"
#define EXAMPLE_ESP_WIFI_PASS "789456123"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

const char TAG[] = "MAIN";

#define LOG_I(...) ESP_LOGI(TAG, __VA_ARGS__)
#define LOG_E(...) ESP_LOGE(TAG, __VA_ARGS__)
#define LOG_W(...) ESP_LOGW(TAG, __VA_ARGS__)

// constexpr i2s_std_config_t I2S_MIC_CONFIG = {
//   .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
//   .sample_rate = SAMPLE_RATE,
//   .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
//   .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
//   .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
//   .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
//   .dma_buf_count = 4,
//   .dma_buf_len = 1024,
//   .use_apll = false,
//   .tx_desc_auto_clear = false,
//   .fixed_mclk = 0
// };

constexpr i2s_std_config_t I2S_MIC_CONFIG = { .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
                                              .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
                                                i2s_data_bit_width_t::I2S_DATA_BIT_WIDTH_32BIT,
                                                i2s_slot_mode_t::I2S_SLOT_MODE_MONO),
                                              .gpio_cfg = {
                                                .mclk = I2S_GPIO_UNUSED,  
            .bclk = gpio_num_t::GPIO_NUM_23,
            .ws   = gpio_num_t::GPIO_NUM_22,
            .dout = gpio_num_t::GPIO_NUM_NC,
            .din  = gpio_num_t::GPIO_NUM_21,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
}};

void
wait_for_button_push()
{
  while (gpio_get_level(BUTTON_PIN) == 0) {

    vTaskDelay(25 / portTICK_PERIOD_MS);
  }

  while (gpio_get_level(BUTTON_PIN) == 1) {

    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}

esp_err_t
record(const std::string_view fname)
{
  ESP_LOGI(TAG, "Starting to record...");

  // initialize the SD card & mount the partition
  SDCard sd_card;
  esp_err_t esp_result = sd_card.Init();
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error initializing the SD card: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  // create & initialize the I2S sampler which samples the microphone
  I2sSampler i2s_sampler;
  esp_result = i2s_sampler.Init(I2S_MIC_CONFIG);
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error initializing the I2S sampler: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  // create a new wave file writer
  WavWriter writer;
  esp_result = writer.Open(fname, MIC_SAMPLE_RATE);
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error opening a file for writing: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  // for (int i = 0; i < 60; i++) {
  //   std::expected<std::array<int16_t, 128>, esp_err_t> result = input->ReadSamples<128>();
  //   if (!result.has_value()) {
  //     LOG_E("%s:%d | Error reading samples: %s", __FILE__, __LINE__, esp_err_to_name(result));
  //   }
  // }

  // keep writing until the user releases the button
  while (gpio_get_level(BUTTON_PIN) == 0) {
    std::vector<int16_t> samples = i2s_sampler.ReadSamples(1024);
    // if (!result.has_value()) {
    //   LOG_E("%s:%d | Error reading samples: %s", __FILE__, __LINE__,
    //   esp_err_to_name(result.error())); continue;
    // }

    // int64_t start = esp_timer_get_time();
    writer.WriteSamples(samples);
    // int64_t end = esp_timer_get_time();
    // ESP_LOGI(TAG, "Wrote %d samples in %lld microseconds", samples_read, end
    // - start );
  }

  // stop the sampler
  i2s_sampler.DeInit();
  // finish the writing
  writer.Close();
  // and de-init the sd card
  sd_card.DeInit();

  ESP_LOGI(TAG, "Finished recording");
  if (gpio_get_level(BUTTON_PIN) == 1) {
    while (gpio_get_level(BUTTON_PIN) == 1) {
      vTaskDelay(25 / portTICK_PERIOD_MS);
    }
    vTaskDelay(25 / portTICK_PERIOD_MS);
  }

  return ESP_OK;
}

void
led(int e)
{
  if (!e)
    gpio_set_level(GPIO_NUM_11, 1);
  else
    gpio_set_level(GPIO_NUM_11, 0);
}

void
setup()
{
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

  // wifi_init_sta();
}

extern "C" void
app_main(void)
{
  // esp_log_level_set("*", ESP_LOG_ERROR);
  // esp_log_level_set("dhcpc", ESP_LOG_INFO);
  // esp_log_level_set("wifi", ESP_LOG_ERROR);
  // esp_log_level_set("esp_netif_handlers", ESP_LOG_INFO);
  // esp_log_level_set("main", ESP_LOG_INFO);
  // esp_log_level_set("I2sSampler", ESP_LOG_INFO);

  setup();

  ESP_LOGI(TAG, "Starting up...");

  Connection wifi_connection;
  wifi_connection.InitWifi(EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);

  // ESP_LOGI(TAG, "Mounting SDCard on /sdcard");

  // ESP_LOGI(TAG, "Creating microphone");

  gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLDOWN_ONLY);

  gpio_set_direction(GPIO_NUM_11, GPIO_MODE_OUTPUT);
  led(0);

  start_connection();

  while (true) {
    // wait for the user to push and hold the button
    // input->start();
    wait_for_button_push();
    led(1);
    record("/sdcard/test.wav");
    led(0);
  }
}
