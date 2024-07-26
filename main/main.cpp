#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <dirent.h>
#include <vector>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
// #include "esp_vfs_fat.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "led_strip.h"

#include "communication.hpp"
#include "ftp_client.hpp"
// #include "FtpClient.h"
#include "i2s_sampler.hpp"
#include "screen_driver.hpp"
#include "sd_card.hpp"
#include "spi2_bus.hpp"
#include "wav_reader.hpp"
#include "wav_writer.hpp"

#include "config.h"
#include "server.h"

#include "settings.hpp"

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

ScreenDriver s_screen_driver;
Connection s_wifi_connection;

/// @brief
/// Startup procedure after a reset.
/// Initializes the NVS, Wi-Fi and synchronizes system time via SNTP.
/// @return `true` if no issues detected, `false` otherwise
bool
StartupSetup();

/// @brief Initializes the NVS. It will be formatted in case memory is full.
/// @return `ESP_OK` in case of success, `esp_err_t` otherwise
esp_err_t
InitNvs();

/// @brief
/// Initializes Wi-Fi and starts the SNTP synchronization,
/// but does no wait for the sync to complete.
/// @return `ESP_OK` in case of success, `esp_err_t` otherwise
esp_err_t
ResumeAfterSleep();

/// @brief
/// Stops the Wi-Fi, configures the recording button to be the wake up source,
/// and enters into the light sleep mode.
/// Automatically invokes `ResumeAfterSleep()` after waking up
/// @return `ESP_OK` in case of success, `esp_err_t` otherwise
esp_err_t
EnterSleep();

/// @brief
/// Initializes needed resources (SPI bus, SD card, screen driver, etc.),
/// records audio into a .wav file with timestamp in its name,
/// then tries to send all stored .wav files to the server,
/// after which they will be deleted if successfully delivered.
/// @return `ESP_OK` in case of success, `esp_err_t` otherwise
esp_err_t
StartRecordingProcess();

/// @brief
/// Initializes the I2S audio sampler,
/// records data from it into a temporary .wav file while the recording button
/// is pushed, renames the temp file to a name which contains the device name
/// and a timestamp.
///
/// Note: Because `ResumeAfterSleep()` does not wait for system time to
/// synchronize via SNTP, file timestamp may be inaccurate if recording is short
/// enough.
/// @return `ESP_OK` if recording was successful & file renamed, `esp_err_t`
/// otherwise
esp_err_t
RecordMicro();

/// @brief Sends all stored .wav files to the server, and deletes them if
/// transfer was a success
/// @return `ESP_OK` if sending was successful, `esp_err_t` if not
esp_err_t
SendStoredFilesToServer();

/// @brief Check if the recording button is pressed
/// @return `true` if recording should be started, `false` otherwise
bool
IsRecButtonPressed();

void
SetLedState(const bool is_enabled);

/// @brief Renames the file at `temp_tile_path`
/// to contain date and time in its name
/// @param temp_file_path path to the temporary .wav
/// file into which the mic recording was stored
/// @return `ESP_OK` if successful, `ESP_FAIL` otherwise
esp_err_t
RenameFile(const std::string_view temp_file_path);

/// @brief Returns an array of file names in the root directory
/// which should be sent to the remote server
/// @param max_amount maximum amount if file names to return
/// @return An array of file names in the root directory
/// which should be sent to the remote server
std::vector<std::string>
GetWavFileNames(const std::size_t max_amount);

/// @brief Checks if the file's extension at `file_path` is .wav
/// @param file_path path to the file to check
/// @return `true` if `file_path` is a .wav file, `false` otherwise
bool
IsWavFile(const std::string_view file_path);

extern "C" void
app_main(void)
{
  LOG_I("Starting up...");

  if (StartupSetup()) {
    LOG_I("Startup has been successful. All systems nominal.");
  } else {
    LOG_W("Something went wrong during startup process. Proceeding anyway...");
  }

  while (true) {
    // Instantly enter sleep mode, which configures the button pin as a
    // wake-up source. When button will be pressed, device will wake-up, and
    // start the recording process
    EnterSleep();

    StartRecordingProcess();
  }
}

bool
StartupSetup()
{
  bool full_success = true;

  // Configure GPIO pins
  esp_err_t esp_result =
    gpio_set_direction(BUTTON_PIN, gpio_mode_t::GPIO_MODE_INPUT);
  esp_result |=
    gpio_set_pull_mode(BUTTON_PIN, gpio_pull_mode_t::GPIO_PULLUP_ONLY);
  if (esp_result != ESP_OK) {
    LOG_E("Failed to configuring GPIO: %s", esp_err_to_name(esp_result));
    full_success = false;
  }

  if (InitNvs() != ESP_OK) {
    LOG_E("NVS initialization failed.");
    full_success = false;
  }

  esp_result =
    s_wifi_connection.InitWifi(EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error initializing Wi-Fi: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    full_success = false;
  }

  esp_result = s_wifi_connection.WaitForSntpSync();
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error waiting for SNTP system time synchronization: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    full_success = false;
  }

  return full_success;
}

esp_err_t
InitNvs()
{
  // Initialize NVS
  esp_err_t esp_result = nvs_flash_init();

  // If there were no issues, just return the success
  if (esp_result == ESP_OK) {
    return ESP_OK;
  }

  // If there issues but they are not related to full memory or new versions,
  // return the failure
  if (esp_result != ESP_ERR_NVS_NO_FREE_PAGES &&
      esp_result != ESP_ERR_NVS_NEW_VERSION_FOUND) {
    LOG_E("%s:%d | Failed to initialize NVS memory: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  // If memory is full or new version is found,
  // NVS should be erased
  if (esp_result == ESP_ERR_NVS_NO_FREE_PAGES) {
    LOG_W("NVS memory is full.");
  } else {
    LOG_W("New NVS version has been found.");
  }

  LOG_W("Erasing NVS memory...");

  // Erase the NVS memory
  esp_result = nvs_flash_erase();
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Failed to erase NVS memory: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  // And re-initialize it
  esp_result = nvs_flash_init();
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Failed to initialize NVS memory after erase: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
  }

  return esp_result;
}

esp_err_t
ResumeAfterSleep()
{
  bool full_success = true;

  esp_err_t esp_result =
    s_wifi_connection.InitWifi(EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error initializing Wi-Fi: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    full_success = false;
  }

  return (full_success ? ESP_OK : ESP_FAIL);
}

esp_err_t
EnterSleep()
{
  esp_err_t esp_result = s_wifi_connection.DeInitWifi();
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Failed to de-initialize Wi-Fi: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
  }

  gpio_wakeup_enable(BUTTON_PIN, gpio_int_type_t::GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  esp_result = esp_light_sleep_start();
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Failed to enter sleep mode: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  LOG_I("Awakened from sleep.");

  esp_result = ResumeAfterSleep();
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Failed resume systems after waking up: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
  }

  return esp_result;
}

esp_err_t
StartRecordingProcess()
{
  LOG_I("Preparing to record...");

  // Initialize the SPI bus
  spi::Spi2Bus& spi_bus = spi::Spi2Bus::GetInstance();
  esp_err_t esp_result =
    spi_bus.InitBus(SPI_PIN_CLK, SPI_PIN_MOSI, SPI_PIN_MISO);
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Failed to initialize the SPI bus: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  s_screen_driver.Init();

  s_screen_driver.Clear();
  s_screen_driver.Clear();
  s_screen_driver.DisplayTextRow(0, "Preparing");
  s_screen_driver.DisplayTextRow(1, "to record...");

  // initialize the SD card & mount the partition
  SDCard sd_card;
  esp_result = sd_card.Init();
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error initializing the SD card: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    s_screen_driver.Clear();
    s_screen_driver.DisplayTextRow(0, "SD card");
    s_screen_driver.DisplayTextRow(1, "error.");
    return esp_result;
  }

  if (RecordMicro() == ESP_OK) {
    esp_result = SendStoredFilesToServer();

    s_screen_driver.Clear();
    s_screen_driver.DisplayTextRow(0, "Recording");
    s_screen_driver.DisplayTextRow(1, "transmission");

    if (esp_result == ESP_OK) {
      s_screen_driver.DisplayTextRow(2, "success.");
    } else {
      s_screen_driver.DisplayTextRow(2, "error.");
      LOG_E("Error transmitting recorded files: %s",
            esp_err_to_name(esp_result));
    }
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
  esp_result = s_screen_driver.DeInit();
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

  return ESP_OK;
}

esp_err_t
RecordMicro()
{
  // create & initialize the I2S sampler which samples the microphone
  I2sSampler i2s_sampler;
  esp_err_t esp_result = i2s_sampler.Init(I2S_MIC_CONFIG);
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error initializing the I2S sampler: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    s_screen_driver.Clear();
    s_screen_driver.DisplayTextRow(0, "Microphone");
    s_screen_driver.DisplayTextRow(1, "error.");
    return esp_result;
  }

  const std::string temp_file_path = SDCard::GetFilePath("temp.wav");

  // create a new wav file writer
  WavWriter writer;
  esp_result = writer.Open(temp_file_path);
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error opening a file for writing: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    s_screen_driver.Clear();
    s_screen_driver.DisplayTextRow(0, "File error.");
    return esp_result;
  }

  // First few samples are a bit rough, it's best to discard them
  i2s_sampler.DiscardSamples(128 * 60);

  LOG_I("Recording...");
  s_screen_driver.Clear();
  s_screen_driver.DisplayTextRow(0, "Recording...");

  // keep writing until the user releases the button
  while (IsRecButtonPressed()) {
    std::vector<int16_t> samples = i2s_sampler.ReadSamples(1024);
    writer.WriteSamples(samples);
  }

  s_screen_driver.Clear();
  s_screen_driver.DisplayTextRow(0, "Recording");
  s_screen_driver.DisplayTextRow(1, "finished.");
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

  esp_result = RenameFile(temp_file_path);
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Error renaming file: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  return ESP_OK;
}

esp_err_t
SendStoredFilesToServer()
{
  // Open FTP server
  LOG_I("ftp server: %s", CONFIG_FTP_SERVER);
  LOG_I("ftp user  : %s", CONFIG_FTP_USER);

  FtpClient ftpClient;

  int connect = ftpClient.ftpClientConnect(CONFIG_FTP_SERVER, CONFIG_FTP_PORT);
  LOG_I("connect=%d", connect);
  if (connect == 0) {
    LOG_E("FTP server connect fail");
    return ESP_FAIL;
  }

  // Login to the FTP server
  int login = ftpClient.ftpClientLogin(CONFIG_FTP_USER, CONFIG_FTP_PASSWORD);
  LOG_I("login=%d", login);
  if (login == 0) {
    LOG_E("FTP server login fail");
    return ESP_FAIL;
  }

  std::vector<std::string> wav_names = GetWavFileNames(8);

  for (auto wav_file : wav_names) {
    const std::string file_path = SDCard::GetFilePath(wav_file);

    LOG_I("Uploading '%.*s'...", file_path.length(), file_path.c_str());

    int result = ftpClient.ftpClientPut(
      file_path.c_str(), wav_file.c_str(), FTP_CLIENT_BINARY);
    if (result != 1) {
      LOG_E("%s:%d | Error uploading '%.*s' to the server.",
            __FILE__,
            __LINE__,
            file_path.length(),
            file_path.c_str());
      return ESP_FAIL;
    }

    result = std::remove(file_path.c_str());
    if (result != 0) {
      LOG_E("%s:%d | Error deleting '%.*s': %d = %s",
            __FILE__,
            __LINE__,
            file_path.length(),
            file_path.c_str(),
            errno,
            strerror(errno));
    }
  }

  ftpClient.ftpClientQuit();

  return ESP_OK;
}

bool
IsRecButtonPressed()
{
  return gpio_get_level(BUTTON_PIN) == 0;
}

esp_err_t
RenameFile(const std::string_view temp_file_path)
{
  constexpr std::size_t file_name_length =
    DEVICE_NAME.length() + sizeof("_0000-00-00_00-00-00.wav");

  // Get current time
  const std::time_t now_time = std::time(nullptr);
  const std::tm* dt = std::localtime(&now_time);

  // Create the buffer, and set it to the needed length
  std::string new_file_name(file_name_length, '\0');

  sprintf(new_file_name.data(),
          "%.*s_%04d-%02d-%02d_%02d-%02d-%02d.wav",
          DEVICE_NAME.length(),
          DEVICE_NAME.data(),
          dt->tm_year + 1900,
          dt->tm_mon,
          dt->tm_mday,
          dt->tm_hour,
          dt->tm_min,
          dt->tm_sec);

  LOG_I("New file name: %s", new_file_name.c_str());

  const std::string new_path = SDCard::GetFilePath(new_file_name);

  if (rename(temp_file_path.data(), new_path.c_str()) != 0) {
    LOG_E("%s:%d | Unable to rename '%.*s' to '%.*s'. errno: %d = %s",
          __FILE__,
          __LINE__,
          temp_file_path.length(),
          temp_file_path.data(),
          new_path.length(),
          new_path.c_str(),
          errno,
          strerror(errno));
    return ESP_FAIL;
  }

  return ESP_OK;
}

std::vector<std::string>
GetWavFileNames(const std::size_t max_amount)
{
  DIR* dir;
  dir = opendir(SDCard::GetMountPoint().data());

  if (dir == nullptr) {
    return {};
  }

  std::size_t file_count = 0;
  dirent* dir_entity;
  std::vector<std::string> wav_names;

  while ((dir_entity = readdir(dir)) != nullptr && file_count < max_amount) {
    // skip the directory entity if it's not a file
    if (dir_entity->d_type != DT_REG) {
      LOG_I("'%s' is not a file. Skipping...", dir_entity->d_name);
      continue;
    }

    const std::string_view file_path(dir_entity->d_name);

    // skip the file if it's not a .wav file
    if (!IsWavFile(file_path)) {
      LOG_I("'%s' is not a .wav file. Skipping...", dir_entity->d_name);
      continue;
    }

    wav_names.push_back(std::string(file_path));

    LOG_I("'%.*s' has been added to the list",
          file_path.length(),
          file_path.data());
  }

  closedir(dir);

  return wav_names;
}

bool
IsWavFile(const std::string_view file_path)
{
  return file_path.substr(file_path.find_last_of(".")) == ".wav";

  // const std::size_t dot_index = file_path.rfind(".");

  // LOG_I("Dot index: %u | extension length: %u",
  //       dot_index,
  //       file_path.length() - dot_index);

  // // if the amount of characters after the dot is not three,
  // // it is definitely not a .wav file
  // if (file_path.length() - dot_index != 4) {
  //   return false;
  // }

  // const std::string_view file_extension = file_path.substr(dot_index + 1,
  // 3);

  // LOG_I("File extension: %.*s", file_extension.length(),
  // file_extension.data());

  // return file_extension == "wav";
}

void
SetLedState(const bool is_enabled)
{
}