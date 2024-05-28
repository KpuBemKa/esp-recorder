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
  const esp_vfs_fat_sdmmc_mount_config_t mount_config = { .format_if_mount_failed = true,
                                                          .max_files = 1,
                                                          .allocation_unit_size = (16 * 1024),
                                                          .disk_status_check_enable = true };

  LOG_I("Initializing SD card...");

  LOG_I("Initializing the SPI bus...");

  const spi_bus_config_t spi_bus_config{
    .mosi_io_num = SD_PIN_MOSI,
    .miso_io_num = SD_PIN_MISO,
    .sclk_io_num = SD_PIN_CLK,

    .data2_io_num = gpio_num_t::GPIO_NUM_NC,
    .data3_io_num = gpio_num_t::GPIO_NUM_NC,
    .data4_io_num = gpio_num_t::GPIO_NUM_NC,
    .data5_io_num = gpio_num_t::GPIO_NUM_NC,
    .data6_io_num = gpio_num_t::GPIO_NUM_NC,
    .data7_io_num = gpio_num_t::GPIO_NUM_NC,

    .max_transfer_sz = 4092, ///< Maximum transfer size, in bytes. Defaults to 4092 if 0 when DMA
                             ///< enabled, or to `SOC_SPI_MAXIMUM_BUFFER_SIZE` if DMA is disabled.
    .flags = SPICOMMON_BUSFLAG_MASTER,
    .isr_cpu_id = esp_intr_cpu_affinity_t::ESP_INTR_CPU_AFFINITY_AUTO
  };

  esp_err_t result = spi_bus_initialize(
    spi_host_device_t::SPI2_HOST, &spi_bus_config, spi_common_dma_t::SPI_DMA_CH_AUTO);
  if (result != ESP_OK) {
    LOG_E(
      "%s:%d | Failed to initialize the SPI bus: %s", __FILE__, __LINE__, esp_err_to_name(result));
    return result;
  }

  // Initialize the SD card and mount the partition
  LOG_I("Mounting the partition...");

  m_host_config = SDSPI_HOST_DEFAULT();

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = SD_PIN_CS;

  result = esp_vfs_fat_sdspi_mount(
    VFS_MOUNT_POINT.data(), &m_host_config, &slot_config, &mount_config, &m_card_info);
  if (result != ESP_OK) {
    LOG_E(
      "%s:%d | Failed to mount the FAT partition: %s", __FILE__, __LINE__, esp_err_to_name(result));
    return result;
  }

#ifdef DEBUG_SD
  LOG_I("SD Card info:");
  sdmmc_card_print_info(stdout, m_card_info);
#endif

  return ESP_OK;
}

esp_err_t
SDCard::DeInit()
{
  const esp_err_t result = esp_vfs_fat_sdcard_unmount(VFS_MOUNT_POINT.data(), m_card_info);

  return result | spi_bus_free(SD_HOST_DEVICE);
}