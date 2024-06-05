#pragma once

#include <string_view>

#include <esp_err.h>

#include "spi_device.hpp"
#include "u8g2.h"

class ScreenDriver
{
public:
  esp_err_t Init();
  esp_err_t DeInit();

  void Clear();

  void DisplayText(const uint16_t x, const uint16_t y, const std::string_view text);
  void DisplayTextRow(const uint16_t row, const std::string_view text);

private:
  esp_err_t ConnectSpi();
  void SetupU8g2();

  static uint8_t u8g2_CallbackSpiByte(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr);
  static uint8_t u8g2_CallbackGpioDelay(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr);

private:
  u8g2_t m_screen_handle;
  inline static spi::SpiDevice m_spi_device;
};