#pragma once

#include <cstdint>
#include <vector>

#include "driver/i2s_std.h"

class I2sSampler
{
public:
  esp_err_t Init(const i2s_std_config_t& i2s_config);
  esp_err_t DeInit();

  std::vector<int16_t> ReadSamples(const std::size_t max_samples);

private:
  i2s_chan_handle_t m_rx_handle;
};
