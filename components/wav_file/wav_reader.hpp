#pragma once

#include <cstdint>
#include <cstdio>
#include <string_view>
#include <vector>

#include "wav_header.hpp"

class WavReader
{
public:
  int Open(const std::string_view file_path);
  void Close();

  int GetSampleRate() { return m_wav_header.sample_rate; }
  std::vector<int16_t> ReadSamples(const std::size_t sample_count);

private:
  FILE* m_fp;

  wav_header_t m_wav_header;
};