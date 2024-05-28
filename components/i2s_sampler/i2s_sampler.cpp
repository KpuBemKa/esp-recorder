#include "i2s_sampler.hpp"

#include "esp_err.h"
#include "esp_log.h"

#include "settings.hpp"

const char TAG[] = "I2S_SAMPLER";

#if DEBUG_MIC
#define LOG_I(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define LOG_I(...)
#endif

#define LOG_E(...) ESP_LOGE(TAG, __VA_ARGS__)
#define LOG_W(...) ESP_LOGW(TAG, __VA_ARGS__)

esp_err_t
I2sSampler::Init(const i2s_std_config_t& i2s_config)
{
  LOG_I("Initializing I2S sampler...");

  // Create a new I2S channel
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  esp_err_t esp_result = i2s_new_channel(&chan_cfg, nullptr, &m_rx_handle);
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Unable to create a new I2S channel: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  // Initialize this new channel in standard mode
  esp_result = i2s_channel_init_std_mode(m_rx_handle, &i2s_config);
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Unable to initialize I2S rx channel: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  // Enable the channel
  esp_result = i2s_channel_enable(m_rx_handle);
  if (esp_result != ESP_OK) {
    LOG_E("%s:%d | Unable to enable I2S RX channel: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  return esp_result;
}

esp_err_t
I2sSampler::DeInit()
{
  esp_err_t esp_result = i2s_channel_disable(m_rx_handle);
  if (esp_result != ESP_OK) {
    LOG_I("%s:%d | Unable to disable I2S RX channel: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  esp_result = i2s_del_channel(m_rx_handle);
  if (esp_result != ESP_OK) {
    LOG_I("%s:%d | Unable to delete I2S RX channel: %s",
          __FILE__,
          __LINE__,
          esp_err_to_name(esp_result));
    return esp_result;
  }

  return esp_result;
}

std::vector<int16_t>
I2sSampler::ReadSamples(const std::size_t max_samples)
{
  // a buffer which will store read data
  std::vector<int16_t> samples_vector;
  samples_vector.resize(max_samples);

  // will store how many bytes were actually read
  std::size_t bytes_read = 0;

  // read the data
  const esp_err_t esp_result =
    i2s_channel_read(m_rx_handle, samples_vector.data(), max_samples, &bytes_read, 1'000);

  LOG_I("%u bytes have been read.", bytes_read);

  // check for errors
  if (esp_result != ESP_OK) {
    LOG_I(
      "%s:%d | Unable to read I2S RX channel: %s", __FILE__, __LINE__, esp_err_to_name(esp_result));
    return {}; // return empty array
  }

  // resize the samples vector
  samples_vector.resize(bytes_read);

  return samples_vector;
}

// esp_err_t
// I2sSampler::ReadSamples(std::span<int16_t> output_buf)
// {
//   const std::size_t bytes_to_read = output_buf.size() * sizeof(int16_t);

//   const esp_err_t esp_result =
//     i2s_channel_read(m_rx_handle, output_buf.data(), bytes_to_read, &bytes_to_read, 1'000);

//   if (esp_result != ESP_OK) {
//     LOG_I(
//       "%s:%d | Unable to read I2S RX channel: %s", __FILE__, __LINE__,
//       esp_err_to_name(esp_result));
//   }

//   return esp_result;
// }

// template<std::size_t N>
// std::expected<std::array<int16_t, N>, esp_err_t>
// I2sSampler::ReadSamples()
// {
//   // read from i2s
//   // int32_t* raw_samples = (int32_t*)malloc(sizeof(int32_t) * count);
//   // size_t bytes_read = 0;
//   // i2s_read(m_i2sPort, raw_samples, sizeof(int32_t) * count, &bytes_read, 100);
//   // int samples_read = bytes_read / sizeof(int32_t);

//   constexpr std::size_t bytes_to_read = N * 2;
//   std::array<int16_t, N> samples;

//   const esp_err_t esp_result =
//     i2s_channel_read(m_rx_handle, &samples.data(), bytes_to_read, bytes_to_read, 1'000);
//   if (esp_result != ESP_OK) {
//     LOG_I(
//       "%s:%d | Unable to read I2S RX channel: %s", __FILE__, __LINE__,
//       esp_err_to_name(esp_result));
//     return std::unexpected(esp_result);
//   }

//   return samples;
// }
