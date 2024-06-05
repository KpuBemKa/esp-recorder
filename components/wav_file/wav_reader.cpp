#include "wav_reader.hpp"

#include "esp_log.h"

const char TAG[] = "WAV_WRITER";

int
WavReader::Open(const std::string_view file_path)
{
  m_fp = fopen(file_path.data(), "rb");
  if (m_fp == nullptr) {
    return -1;
  }

  // read the WAV header
  fread(&m_wav_header, sizeof(wav_header_t), 1, m_fp);

  // sanity check the bit depth
  // if (m_wav_header.bit_depth != 16) {
  //   ESP_LOGE(TAG, "ERROR: bit depth %d is not supported\n", m_wav_header.bit_depth);
  // }

  if (m_wav_header.num_channels != 1) {
    ESP_LOGE(TAG, "ERROR: channels %d is not supported\n", m_wav_header.num_channels);
  }

  ESP_LOGI(TAG,
           "fmt_chunk_size=%ld, audio_format=%d, num_channels=%d, "
           "sample_rate=%ld, sample_alignment=%d, bit_depth=%d, data_bytes=%d\n",
           m_wav_header.fmt_chunk_size,
           m_wav_header.audio_format,
           m_wav_header.num_channels,
           m_wav_header.sample_rate,
           m_wav_header.sample_alignment,
           m_wav_header.bits_per_sample,
           m_wav_header.data_bytes);

  return 0;
}

void
WavReader::Close()
{
  fclose(m_fp);
}

std::vector<int16_t>
WavReader::ReadSamples(const std::size_t sample_count)
{
  std::vector<int16_t> result;
  result.resize(sample_count * sizeof(int16_t));

  fread(result.data(), sizeof(uint8_t), result.size(), m_fp);

  return result;
}