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

#include "config.h"
#include "server.h"

static const char* TAG = "main";

extern int32_t currentmax;

#define EXAMPLE_ESP_WIFI_SSID "tenevoi"
#define EXAMPLE_ESP_WIFI_PASS "dimadima"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

void
wait_for_button_push()
{
  while (gpio_get_level(GPIO_BUTTON) == 0) {

    vTaskDelay(25 / portTICK_PERIOD_MS);
  }

  while (gpio_get_level(GPIO_BUTTON) == 1) {

    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}

void
record(I2sSampler* input, const std::string_view fname)
{

  int16_t* samples = (int16_t*)malloc(sizeof(int16_t) * 1024);
  ESP_LOGI(TAG, "Start recording");
  // input->start();

  // create a new wave file writer
  WavWriter writer;
  writer.Open(fname, input->sample_rate());

  for (int i = 0; i < 60; i++) {
    input->read(samples, 128, true);
  }

  // keep writing until the user releases the button
  while (gpio_get_level(GPIO_BUTTON) == 0) {
    int samples_read = input->read(samples, 1024, true);
    // int64_t start = esp_timer_get_time();
    writer.WriteSamples(std::span<int16_t>(samples, (std::size_t)samples_read));
    // int64_t end = esp_timer_get_time();
    // ESP_LOGI(TAG, "Wrote %d samples in %lld microseconds", samples_read, end
    // - start );
  }
  // stop the input
  input->stop();
  // and finish the writing
  writer.Close();

  free(samples);

  ESP_LOGI(TAG, "Finished recording");
  if (gpio_get_level(GPIO_BUTTON) == 1) {
    while (gpio_get_level(GPIO_BUTTON) == 1) {
      vTaskDelay(25 / portTICK_PERIOD_MS);
    }
    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}

void
led(int e)
{
  if (!e)
    gpio_set_level(GPIO_NUM_11, 1);
  else
    gpio_set_level(GPIO_NUM_11, 0);
}

static int s_retry_num = 0;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static void
event_handler(void* arg,
              esp_event_base_t event_base,
              int32_t event_id,
              void* event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "retry to connect to the AP");
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(TAG, "connect to the AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

void
wifi_init_sta(void)
{
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
    WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
    IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

  wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            //.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK,
            //.sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            //.sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_sta finished.");

  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or
   * connection failed for the maximum number of re-tries (WIFI_FAIL_BIT). The
   * bits are set by event_handler() (see above) */
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE,
                                         pdFALSE,
                                         portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we
   * can test which event actually happened. */
  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG,
             "connected to ap SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID,
             EXAMPLE_ESP_WIFI_PASS);
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG,
             "Failed to connect to SSID:%s, password:%s",
             EXAMPLE_ESP_WIFI_SSID,
             EXAMPLE_ESP_WIFI_PASS);
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }
}

void
setup()
{

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

  wifi_init_sta();
}

extern "C" void
app_main(void)
{
  esp_log_level_set("*", ESP_LOG_ERROR);
  esp_log_level_set("dhcpc", ESP_LOG_INFO);
  esp_log_level_set("wifi", ESP_LOG_ERROR);
  esp_log_level_set("esp_netif_handlers", ESP_LOG_INFO);
  esp_log_level_set("main", ESP_LOG_INFO);
  esp_log_level_set("I2sSampler", ESP_LOG_INFO);

  setup();

  ESP_LOGI(TAG, "Starting up");

  ESP_LOGI(TAG, "Mounting SDCard on /sdcard");
  new SDCard("/sdcard", PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS);

  ESP_LOGI(TAG, "Creating microphone");

  I2sSampler* input = new I2sSampler(I2S_NUM_0, i2s_mic_pins, i2s_mic_Config);

  gpio_set_direction(GPIO_BUTTON, GPIO_MODE_INPUT);
  gpio_set_pull_mode(GPIO_BUTTON, GPIO_PULLDOWN_ONLY);

  gpio_set_direction(GPIO_NUM_11, GPIO_MODE_OUTPUT);
  led(0);

  send_to_server();
  while (true) {
    // wait for the user to push and hold the button
    input->start();
    wait_for_button_push();
    led(1);
    record(input, "/sdcard/test.wav");
    led(0);
  }
}
