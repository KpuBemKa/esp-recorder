#pragma once

#include <string_view>

#include "driver/sdmmc_types.h"
#include "driver/sdspi_host.h"
#include "hal/gpio_types.h"

class SDCard
{
public:
  esp_err_t Init();
  esp_err_t DeInit();

private:
  sdmmc_card_t* m_card_info;
  sdmmc_host_t m_host_config;
};