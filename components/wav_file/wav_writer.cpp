#include "wav_writer.hpp"

#include <cerrno>
#include <cstring>

#include "esp_log.h"

#include "settings.hpp"

const char TAG[] = "SD_CARD";

#if DEBUG_WAV
#define LOG_I(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define LOG_I(...)
#endif

#define LOG_E(...) ESP_LOGE(TAG, __VA_ARGS__)
#define LOG_W(...) ESP_LOGW(TAG, __VA_ARGS__)

esp_err_t
WavWriter::Open(
  const std::string_view file_path /* , const std::size_t sample_rate */)
{
  LOG_I("Opening file '%.*s'...", file_path.size(), file_path.data());

  m_fp = fopen(file_path.data(), "wb");

  if (m_fp == nullptr) {
    perror("");
    return ESP_ERR_INVALID_STATE;
  }

  // m_header.sample_rate = sample_rate;
  // write out the header - we'll fill in some of the blanks later
  const std::size_t written =
    std::fwrite(&m_header, sizeof(wav_header_t), 1, m_fp);
  if (written != 1) {
    LOG_E("%s:%d | Error writing the WAV header:", __FILE__, __LINE__);
    perror("");
    return ESP_FAIL;
  }

  m_file_size = sizeof(wav_header_t);

  return ESP_OK;
}

esp_err_t
WavWriter::Close()
{
  if (m_fp != nullptr) {
    return FinishAndClose();
  } else {
    return ESP_OK;
  }
}

WavWriter::~WavWriter()
{
  Close();
}

void
WavWriter::WriteSamples(const std::span<int16_t> samples)
{
  // write the samples and keep track of the file size so far
  const std::size_t written =
    fwrite(samples.data(), sizeof(samples[0]), samples.size(), m_fp);
  if (written != samples.size()) {
    LOG_E("%s:%d | Error writing samples. Samples to write: %u | written: %u",
          __FILE__,
          __LINE__,
          samples.size(),
          written);
    perror("");
  }

  m_file_size += sizeof(samples[0]) * written;
}

esp_err_t
WavWriter::FinishAndClose()
{
  ESP_LOGI(TAG, "Finished wav file size: %d", m_file_size);

  // now fill in the header with the correct information and write it again
  m_header.data_bytes = m_file_size - sizeof(wav_header_t);
  m_header.wav_size = m_file_size - 8;

  fseek(m_fp, 0, SEEK_SET);
  fwrite(&m_header, sizeof(m_header), 1, m_fp);

  if (fclose(m_fp) != 0) {
    LOG_E("%s:%d | Unable to close the file. errno: %d = %s",
          __FILE__,
          __LINE__,
          errno,
          std::strerror(errno));
    return ESP_FAIL;
  }

  m_fp = nullptr;

  return ESP_OK;
}