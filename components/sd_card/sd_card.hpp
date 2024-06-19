#pragma once

#include <string>
#include <string_view>

#include "driver/sdmmc_types.h"
#include "driver/sdspi_host.h"
#include "hal/gpio_types.h"

class SDCard
{
public:
  esp_err_t Init();
  esp_err_t DeInit();

  ~SDCard();

  /// @brief Creates a full file path string for `file_name`, which includes SD card VFS mount point
  /// @param file_name file name
  /// @return file path
  static std::string GetFilePath(const std::string_view file_name);

  static std::string_view GetMountPoint();

private:
  bool m_is_init = false;
  
  sdmmc_card_t* m_card_info;
  sdmmc_host_t m_host_config;
};