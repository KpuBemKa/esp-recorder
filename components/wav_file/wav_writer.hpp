#pragma once

#include <cstdint>
#include <cstdio>
#include <span>
#include <string_view>

#include "wav_header.hpp"

class WavWriter
{
public:
  int Open(const std::string_view file_path, const std::size_t sample_rate);
  void Close();

  void WriteSamples(const std::span<int16_t> samples);

private:
  void FinishAndClose();

private:
  FILE* m_fp;

  wav_header_t m_header;
  int m_file_size;
};