#include "wav_writer.hpp"

#include "esp_log.h"

const char TAG[] = "WAV_WRITER";

int
WavWriter::Open(const std::string_view file_path, const std::size_t sample_rate)
{
  m_fp = fopen(file_path.data(), "wb");

  if (m_fp == nullptr) {
    return -1;
  }

  m_header.sample_rate = sample_rate;
  // write out the header - we'll fill in some of the blanks later
  int written = fwrite(&m_header, sizeof(wav_header_t), 1, m_fp);

  m_file_size = sizeof(wav_header_t);

  return written;
}

void
WavWriter::Close()
{
  FinishAndClose();
}

void
WavWriter::WriteSamples(const std::span<int16_t> samples)
{
  // write the samples and keep track of the file size so far
  fwrite(samples.data(), sizeof(int16_t), samples.size(), m_fp);
  m_file_size += sizeof(int16_t) * samples.size();
}

void
WavWriter::FinishAndClose()
{
  ESP_LOGI(TAG, "Finished wav file size: %d", m_file_size);

  // now fill in the header with the correct information and write it again
  m_header.data_bytes = m_file_size - sizeof(wav_header_t);
  m_header.wav_size = m_file_size - 8;

  fseek(m_fp, 0, SEEK_SET);
  fwrite(&m_header, sizeof(m_header), sizeof(uint8_t), m_fp);
}