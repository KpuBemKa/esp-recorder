#include <cstring>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
// #include "esp_vfs_fat.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "led_strip.h"

#include "communication.hpp"
#include "ftp_client.hpp"
#include "i2s_sampler.hpp"
#include "screen_driver.hpp"
#include "sd_card.hpp"
#include "spi2_bus.hpp"
#include "wav_reader.hpp"
#include "wav_writer.hpp"

#include "config.h"
#include "server.h"

#include "settings.hpp"

extern int32_t currentmax;

#define EXAMPLE_ESP_WIFI_SSID "MARS"
#define EXAMPLE_ESP_WIFI_PASS "789456123"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define CONFIG_FTP_SERVER "192.168.50.111"
#define CONFIG_FTP_PORT 21
#define CONFIG_FTP_USER "esp-recordings"
#define CONFIG_FTP_PASSWORD "Admin0308"

const char TAG[] = "MAIN";

#define LOG_I(...) ESP_LOGI(TAG, __VA_ARGS__)
#define LOG_E(...) ESP_LOGE(TAG, __VA_ARGS__)
#define LOG_W(...) ESP_LOGW(TAG, __VA_ARGS__)

constexpr i2s_std_config_t I2S_MIC_CONFIG = { 
  .clk_cfg = {
    .sample_rate_hz = MIC_SAMPLE_RATE,
    .clk_src = I2S_CLK_SRC_DEFAULT,
    .ext_clk_freq_hz = 0,
    .mclk_multiple = I2S_MCLK_MULTIPLE_256,
  },

  // .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
  //   i2s_data_bit_width_t::I2S_DATA_BIT_WIDTH_32BIT,
  //   i2s_slot_mode_t::I2S_SLOT_MODE_MONO),
  .slot_cfg = {
    .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT, 
    .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO, 
    .slot_mode = I2S_SLOT_MODE_STEREO, 
    .slot_mask = I2S_STD_SLOT_LEFT,
    .ws_width = I2S_DATA_BIT_WIDTH_32BIT, 
    .ws_pol = false, 
    .bit_shift = true, 
    .left_align = true, 
    .big_endian = false, 
    .bit_order_lsb = false 
  },
    
  .gpio_cfg = {
    .mclk = I2S_GPIO_UNUSED,  
    .bclk = I2S_MIC_SERIAL_CLOCK,
    .ws   = I2S_MIC_WORD_SELECT,
    .dout = gpio_num_t::GPIO_NUM_NC,
    .din  = I2S_MIC_SERIAL_DATA,
    .invert_flags = {
      .mclk_inv = false,
      .bclk_inv = false,
      .ws_inv   = false,
    },
  }
};

ScreenDriver screen_driver;

/// @brief Starts the recording process
esp_err_t
start_recording_process();

/// @brief Records data from the microphone to a wav file
/// @param file_name file name into which to record
/// @return `ESP_OK` if recording was successful, `esp_err_t` if not
esp_err_t
record_micro(const std::string_view file_name);

/// @brief Sends all stored files to the server, and deletes them if transfer was a success
/// @return `ESP_OK` if sending was successful, `esp_err_t` if not
esp_err_t
send_files_to_server();

bool
IsRecButtonPressed()
{
  return gpio_get_level(BUTTON_PIN) == 0;
}

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

// esp_err_t
// mount_storage()
// {
//   LOG_I("Initializing SPIFFS...");

//   esp_vfs_spiffs_conf_t conf = {
//     .base_path = VFS_MOUNT_POINT.data(),
//     .partition_label = nullptr,
//     .max_files = 1, // This sets the maximum number of files that can be open at the same time
//     .format_if_mount_failed = true
//   };

//   esp_err_t ret = esp_vfs_spiffs_register(&conf);
//   if (ret != ESP_OK) {
//     if (ret == ESP_FAIL) {
//       LOG_E("Failed to mount or format filesystem");
//     } else if (ret == ESP_ERR_NOT_FOUND) {
//       LOG_E("Failed to find SPIFFS partition");
//     } else {
//       LOG_E("Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
//     }
//     return ret;
//   }

//   size_t total = 0, used = 0;
//   ret = esp_spiffs_info(NULL, &total, &used);
//   if (ret != ESP_OK) {
//     LOG_E("Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
//     return ret;
//   }

//   LOG_I("Partition size: total: %d, used: %d", total, used);

//   return ESP_OK;
// }

esp_err_t
recordd(const std::string_view fname)
{

  LOG_I("Preparing to record...");
  screen_driver.Clear();
  screen_driver.DisplayTextRow(0, "Preparing");
  screen_driver.DisplayTextRow(1, "to record...");

  // initialize the SD card & mount the partition
  SDCard sd_card;
  esp_err_t esp_result = sd_card.Init();
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error initializing the SD card: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    screen_driver.Clear();
    screen_driver.DisplayTextRow(0, "SD card");
    screen_driver.DisplayTextRow(1, "error.");
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
    screen_driver.Clear();
    screen_driver.DisplayTextRow(0, "Microphone");
    screen_driver.DisplayTextRow(1, "error.");
    return esp_result;
  }

  // create a new wav file writer
  WavWriter writer;
  esp_result = writer.Open(SDCard::GetFilePath(fname) /* , MIC_SAMPLE_RATE */);
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error opening a file for writing: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    screen_driver.Clear();
    screen_driver.DisplayTextRow(0, "File error.");
    return esp_result;
  }

  LOG_I("Recording...");
  screen_driver.Clear();
  screen_driver.DisplayTextRow(0, "Recording...");

  // First few samples are a bit rough, it's best to discard them
  i2s_sampler.DiscardSamples(128 * 60);

  // keep writing until the user releases the button
  while (IsRecButtonPressed()) {
    std::vector<int16_t> samples = i2s_sampler.ReadSamples(1024);
    writer.WriteSamples(samples);
  }

  screen_driver.Clear();
  screen_driver.DisplayTextRow(0, "Recording");
  screen_driver.DisplayTextRow(1, "finished.");
  LOG_I("Finished recording.");

  // stop the sampler
  esp_result = i2s_sampler.DeInit();
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error de-initializing the I2S sampler: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  // finish the writing
  writer.Close();
  // esp_result = writer.Close();
  // if (esp_result != ESP_OK) {
  //   LOG_E("%s:%d | Error closing recorded .wav file: %s",
  //         __FILE__,
  //         __LINE__,
  //         esp_err_to_name(esp_result));
  //   return esp_result;
  // }

  // and de-init the sd card
  esp_result = sd_card.DeInit();
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error closing de-initializing the SD card: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  LOG_I("Released used resources.");

  // if (gpio_get_level(BUTTON_PIN) == 1) {
  //   while (gpio_get_level(BUTTON_PIN) == 1) {
  //     vTaskDelay(25 / portTICK_PERIOD_MS);
  //   }
  //   vTaskDelay(25 / portTICK_PERIOD_MS);
  // }

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

  // LOG_I("ESP_WIFI_MODE_STA");

  // wifi_init_sta();
}

// void
// task_test_SSD1309()
// {
//   u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
//   u8g2_esp32_hal.bus.spi.clk = SCREEN_PIN_SCL;
//   u8g2_esp32_hal.bus.spi.mosi = SCREEN_PIN_SDA;
//   u8g2_esp32_hal.bus.spi.cs = SCREEN_PIN_CS;
//   u8g2_esp32_hal.dc = SCREEN_PIN_DC;
//   u8g2_esp32_hal.reset = SCREEN_PIN_RESET;
//   u8g2_esp32_hal_init(u8g2_esp32_hal);

//   u8g2_t u8g2; // a structure which will contain all the data for one display
//   u8g2_Setup_ssd1309_128x64_noname2_f(&u8g2,
//                                       U8G2_R0,
//                                       u8g2_esp32_spi_byte_cb,
//                                       u8g2_esp32_gpio_and_delay_cb); // init u8g2 structure

//   u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in
//                            // sleep mode after this,

//   u8g2_SetPowerSave(&u8g2, 0); // wake up display
//   u8g2_ClearBuffer(&u8g2);
//   u8g2_DrawBox(&u8g2, 10, 20, 20, 30);
//   u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
//   u8g2_DrawStr(&u8g2, 0, 15, "Hello World!");
//   u8g2_SendBuffer(&u8g2);

//   LOG_I("All done!");

//   // vTaskDelete(NULL);
// }

static uint8_t s_led_state = 0;
static led_strip_handle_t led_strip;

void
blink_led(void)
{
  /* If the addressable LED is enabled */
  if (s_led_state) {
    /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
    led_strip_set_pixel(led_strip, 0, 16, 16, 16);
    /* Refresh the strip to send data */
    led_strip_refresh(led_strip);
  } else {
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
  }
}

void
configure_led(void)
{
  /* LED strip initialization with the GPIO and pixels number*/
  led_strip_config_t strip_config = { .strip_gpio_num = ARGB_LED_PIN,
                                      .max_leds = 1, // at least one LED on board
                                      .led_pixel_format = led_pixel_format_t::LED_PIXEL_FORMAT_GRB,
                                      .led_model = led_model_t::LED_MODEL_WS2812,
                                      .flags{ .invert_out = false } };
  // #if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
  // led_strip_rmt_config_t rmt_config{ .clk_src = rmt_clock_source_t::RMT_CLK_SRC_DEFAULT,
  //                                    .resolution_hz = 10 * 1000 * 1000, // 10MHz
  //                                    .mem_block_symbols = 0,
  //                                    .flags{ .with_dma = false } };
  // ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  // #elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
  led_strip_spi_config_t spi_config{ .clk_src = spi_clock_source_t::SPI_CLK_SRC_DEFAULT,
                                     .spi_bus = SPI2_HOST,
                                     .flags{ .with_dma = true }

  };
  ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
  // #else
  // #error "unsupported LED strip backend"
  // #endif
  /* Set all LED off to clear all pixels */
  led_strip_clear(led_strip);
}

extern "C" void
app_main(void)
{
  // esp_log_level_set("u8g2_hal", ESP_LOG_DEBUG);
  // esp_log_level_set("dhcpc", ESP_LOG_INFO);
  // esp_log_level_set("wifi", ESP_LOG_ERROR);
  // esp_log_level_set("esp_netif_handlers", ESP_LOG_INFO);
  // esp_log_level_set("main", ESP_LOG_INFO);
  // esp_log_level_set("I2sSampler", ESP_LOG_INFO);

  LOG_I("Starting up...");

  setup();

  // gpio_set_direction(SD_PIN_CS, gpio_mode_t::GPIO_MODE_OUTPUT);
  // gpio_pulldown_dis(SD_PIN_CS);
  // gpio_pullup_en(SD_PIN_CS);

  // configure_led();
  // led_strip_set_pixel(led_strip, 0, 16, 16, 16);
  /* Refresh the strip to send data */
  // led_strip_refresh(led_strip);

  // SDCard sd_card;
  // esp_err_t esp_result = sd_card.Init();
  // if (esp_result != ESP_OK) {
  //   LOG_E("%s:%d | Error initializing the SD card: %s",
  //         __FILE__,
  //         __LINE__,
  //         esp_err_to_name(esp_result));
  // }

  // task_test_SSD1309();

  // LOG_I("Mounting SDCard on /sdcard");

  // LOG_I("Creating microphone");

  gpio_set_direction(gpio_num_t::GPIO_NUM_13, gpio_mode_t::GPIO_MODE_OUTPUT);

  gpio_set_direction(BUTTON_PIN, gpio_mode_t::GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_PIN, gpio_pull_mode_t::GPIO_PULLUP_ONLY);

  while (true) {
    if (!IsRecButtonPressed()) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    start_recording_process();
  }
}

esp_err_t
start_recording_process()
{
  LOG_I("Preparing to record...");

  // Initialize the SPI bus
  spi::Spi2Bus& spi_bus = spi::Spi2Bus::GetInstance();
  esp_err_t esp_result = spi_bus.InitBus(SPI_PIN_CLK, SPI_PIN_MOSI, SPI_PIN_MISO);
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Failed to initialize the SPI bus: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  screen_driver.Init();

  screen_driver.Clear();
  screen_driver.Clear();
  screen_driver.DisplayTextRow(0, "Preparing");
  screen_driver.DisplayTextRow(1, "to record...");

  // initialize the SD card & mount the partition
  SDCard sd_card;
  esp_result = sd_card.Init();
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error initializing the SD card: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    screen_driver.Clear();
    screen_driver.DisplayTextRow(0, "SD card");
    screen_driver.DisplayTextRow(1, "error.");
    return esp_result;
  }

  vTaskDelay(pdMS_TO_TICKS(1'000));

  // if (record_micro("test.wav") == ESP_OK) {
  //   esp_result = send_files_to_server();

  //   screen_driver.Clear();
  //   screen_driver.DisplayTextRow(0, "Recording");
  //   screen_driver.DisplayTextRow(1, "transmission");

  //   if (esp_result == ESP_OK) {
  //     screen_driver.DisplayTextRow(2, "success.");
  //   } else {
  //     LOG_E("Error transmitting recorded files: %s", esp_err_to_name(esp_result));
  //     screen_driver.DisplayTextRow(2, "error.");
  //   }
  // }

  esp_result = send_files_to_server();

  screen_driver.Clear();
  screen_driver.DisplayTextRow(0, "Recording");
  screen_driver.DisplayTextRow(1, "transmission");

  if (esp_result == ESP_OK) {
    screen_driver.DisplayTextRow(2, "success.");
  } else {
    LOG_E("Error transmitting recorded files: %s", esp_err_to_name(esp_result));
    screen_driver.DisplayTextRow(2, "error.");
  }

  // de-init the sd card
  esp_result = sd_card.DeInit();
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error de-initializing the SD card: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  // de-init the screen
  esp_result = screen_driver.DeInit();
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error de-initializing the screen: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  // de-init the SPI bus
  esp_result = spi_bus.DeInitBus();
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error de-initializing the SPI bus: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  LOG_I("Released used resources.");

  // if (gpio_get_level(BUTTON_PIN) == 1) {
  //   while (gpio_get_level(BUTTON_PIN) == 1) {
  //     vTaskDelay(25 / portTICK_PERIOD_MS);
  //   }
  //   vTaskDelay(25 / portTICK_PERIOD_MS);
  // }

  return ESP_OK;
}

esp_err_t
record_micro(const std::string_view file_name)
{
  // create & initialize the I2S sampler which samples the microphone
  I2sSampler i2s_sampler;
  esp_err_t esp_result = i2s_sampler.Init(I2S_MIC_CONFIG);
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error initializing the I2S sampler: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    screen_driver.Clear();
    screen_driver.DisplayTextRow(0, "Microphone");
    screen_driver.DisplayTextRow(1, "error.");
    return esp_result;
  }

  // create a new wav file writer
  WavWriter writer;
  esp_result = writer.Open(SDCard::GetFilePath("test.wav") /* , MIC_SAMPLE_RATE */);
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error opening a file for writing: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    screen_driver.Clear();
    screen_driver.DisplayTextRow(0, "File error.");
    return esp_result;
  }

  // First few samples are a bit rough, it's best to discard them
  i2s_sampler.DiscardSamples(128 * 60);

  LOG_I("Recording...");
  screen_driver.Clear();
  screen_driver.DisplayTextRow(0, "Recording...");

  // keep writing until the user releases the button
  while (IsRecButtonPressed()) {
    std::vector<int16_t> samples = i2s_sampler.ReadSamples(1024);
    writer.WriteSamples(samples);
  }

  screen_driver.Clear();
  screen_driver.DisplayTextRow(0, "Recording");
  screen_driver.DisplayTextRow(1, "finished.");
  LOG_I("Finished recording.");

  // stop the sampler
  esp_result = i2s_sampler.DeInit();
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error de-initializing the I2S sampler: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  // finish the writing
  writer.Close();

  return ESP_OK;
}

esp_err_t
send_files_to_server()
{
  Connection wifi_connection;
  const esp_err_t esp_result =
    wifi_connection.InitWifi(EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error initializing Wi-Fi: %s", __FILE__, __LINE__, esp_err_to_name(esp_result));
    return esp_result;
  }

  // Open FTP server
  LOG_I("ftp server: %s", CONFIG_FTP_SERVER);
  LOG_I("ftp user  : %s", CONFIG_FTP_USER);
  // static NetBuf_t* ftpClientNetBuf = NULL;
  FtpClient ftpClient;
  // int connect = ftpClient->ftpClientConnect(CONFIG_FTP_SERVER, 21, &ftpClientNetBuf);
  // int connect = ftpClient->ftpClientConnect(CONFIG_FTP_SERVER, 2121, &ftpClientNetBuf);
  int connect = ftpClient.ftpClientConnect(CONFIG_FTP_SERVER, CONFIG_FTP_PORT);
  LOG_I("connect=%d", connect);
  if (connect == 0) {
    LOG_E("FTP server connect fail");
    return ESP_FAIL;
  }

  // Login FTP server
  int login = ftpClient.ftpClientLogin(CONFIG_FTP_USER, CONFIG_FTP_PASSWORD);
  LOG_I("login=%d", login);
  if (login == 0) {
    LOG_E("FTP server login fail");
    return ESP_FAIL;
  }

  // Remote Directory
  // char line[128];
  // ftpClient->ftpClientDir(outFileName, "/", ftpClientNetBuf);
  // ftpClient.ftpClientDir(outFileName, ".", ftpClientNetBuf);
  // FILE* f = fopen(outFileName, "r");
  // if (f == NULL) {
  //   LOG_E("Failed to open file for reading");
  //   return;
  // }
  // while (fgets(line, sizeof(line), f) != NULL) {
  //   int len = strlen(line);
  //   line[len - 1] = 0;
  //   LOG_I("%s", line);
  // }
  // fclose(f);
  // LOG_I("");

  // Use POSIX and C standard library functions to work with files.
  // Create file
  // f = fopen(srcFileName, "w");
  // if (f == NULL) {
  //   LOG_E("Failed to open file for writing");
  //   return;
  // }
  // fprintf(f, "Hello World!\n");
  // fclose(f);
  // LOG_I("Wrote the text on %s", srcFileName);

  const std::string wav_filepath = SDCard::GetFilePath("test.wav");

  // Put file to FTP server
  ftpClient.ftpClientPut(wav_filepath.c_str(), "test.wav", FTP_CLIENT_BINARY);
  LOG_I("ftpClientPut %s ---> %s", wav_filepath.c_str(), "test.wav");

  ftpClient.ftpClientQuit();

  wifi_connection.DeInitWifi();

  return ESP_OK;
}