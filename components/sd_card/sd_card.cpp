#include "sd_card.hpp"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "settings.hpp"
#include "spi2_bus.hpp"

const char TAG[] = "SD_CARD";

#if DEBUG_SD
#define LOG_I(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define LOG_I(...)
#endif

#define LOG_E(...) ESP_LOGE(TAG, __VA_ARGS__)
#define LOG_W(...) ESP_LOGW(TAG, __VA_ARGS__)

constexpr spi_host_device_t SD_HOST_DEVICE = spi_host_device_t::SPI2_HOST;

esp_err_t
SDCard::Init()
{
  constexpr esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = true,
    .max_files = 1,
    .allocation_unit_size = (16 * 1024),
    .disk_status_check_enable = true
  };

  LOG_I("Initializing SD card...");

  if (m_is_init) {
    LOG_I("SD card is already initialized.");
    return ESP_OK;
  }

  // Ensure that SPI bus is initialized
  if (!spi::Spi2Bus::GetInstance().IsInit()) {
    LOG_E("SPI bus is not initialized. SD card cannot be mounted.");
    return ESP_ERR_INVALID_STATE;
  }

  // Initialize the SD card and mount the partition
  LOG_I("Mounting the partition...");

  m_host_config = SDSPI_HOST_DEFAULT();
  // m_host_config.max_freq_khz = SDMMC_FREQ_PROBING;

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = SD_PIN_CS;

  esp_err_t result = esp_vfs_fat_sdspi_mount(VFS_MOUNT_POINT.data(),
                                             &m_host_config,
                                             &slot_config,
                                             &mount_config,
                                             &m_card_info);
  if (result != ESP_OK) {
    const esp_err_t result =
      esp_vfs_fat_sdcard_unmount(VFS_MOUNT_POINT.data(), m_card_info);
    if (result != ESP_OK) {
      LOG_E("%s:%d | Error unmounting the partition: %s",
            __FILE__,
            __LINE__,
            esp_err_to_name(result));
      return result;
    }

    LOG_E("%s:%d | Failed to mount the FAT partition: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(result));
    return result;
  }

#ifdef DEBUG_SD
  LOG_I("SD Card info:");
  sdmmc_card_print_info(stdout, m_card_info);
#endif

  m_is_init = true;

  return ESP_OK;
}

esp_err_t
SDCard::DeInit()
{
  LOG_I("De-initializing the SD card...");

  if (!m_is_init) {
    LOG_I("SD card is not initialized. Skipping the de-initialization...");
    return ESP_OK;
  }

  const esp_err_t result =
    esp_vfs_fat_sdcard_unmount(VFS_MOUNT_POINT.data(), m_card_info);
  if (result != ESP_OK) {
    LOG_E("%s:%d | Error unmounting the partition: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(result));
    return result;
  }

  m_is_init = false;

  return result;

  // result = spi_bus_free(SD_HOST_DEVICE);
  // if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
  //   LOG_E("%s:%d | Failed to free the SPI bus: %s", __FILE__, __LINE__,
  //   esp_err_to_name(result)); return result;
  // }
}

SDCard::~SDCard()
{
  DeInit();
}

std::string
SDCard::GetFilePath(const std::string_view file_name)
{
  std::string file_path;
  file_path.reserve(VFS_MOUNT_POINT.size() + sizeof('/') + file_path.size());

  file_path.append(VFS_MOUNT_POINT).append(1, '/').append(file_name);

  return file_path;
}

std::string_view
SDCard::GetMountPoint()
{
  return VFS_MOUNT_POINT;
}