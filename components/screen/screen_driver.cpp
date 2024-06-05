#include "screen_driver.hpp"

#include <driver/gpio.h>
#include <esp_log.h>

#include "u8g2.h"

#include "settings.hpp"
#include "spi2_bus.hpp"

const char TAG[] = "SCREEN";

#if DEBUG_SCREEN
#define LOG_I(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define LOG_I(...)
#endif

#define LOG_E(...) ESP_LOGE(TAG, __VA_ARGS__)
#define LOG_W(...) ESP_LOGW(TAG, __VA_ARGS__)

const uint8_t* MAIN_FONT = u8g2_font_ncenB14_tr;
constexpr uint16_t row_height = 14; // font size is 14

esp_err_t
ScreenDriver::Init()
{
  // Connect the SPI bus
  if (ConnectSpi() != ESP_OK) {
    return ESP_FAIL;
  }

  SetupU8g2();

  return ESP_OK;
}

esp_err_t
ScreenDriver::DeInit()
{
  u8g2_SetPowerSave(&m_screen_handle, 1);

  const esp_err_t result = m_spi_device.DeInit();
  if (result != ESP_OK) {
    LOG_E("%s:%d | Error connecting the SPI bus: %s", __FILE__, __LINE__, esp_err_to_name(result));
  }

  return result;
}

void
ScreenDriver::Clear()
{
  u8g2_ClearDisplay(&m_screen_handle);
  u8g2_ClearBuffer(&m_screen_handle);
}

void
ScreenDriver::DisplayText(const uint16_t x, const uint16_t y, const std::string_view text)
{
  u8g2_SetFont(&m_screen_handle, MAIN_FONT);
  u8g2_DrawStr(&m_screen_handle, x, y, text.data());

  u8g2_SendBuffer(&m_screen_handle);
}

void
ScreenDriver::DisplayTextRow(const uint16_t row, const std::string_view text)
{
  constexpr uint16_t x_offset = 1;
  const uint16_t y_offset = 1 + row * 2; // add two pixela between rows

  DisplayText(x_offset, (row + 1) * row_height + y_offset, text);
}

esp_err_t
ScreenDriver::ConnectSpi()
{
  if (!m_spi_device.IsBusInit()) {
    LOG_E("Spi bus is not initialized.");
    return ESP_ERR_INVALID_STATE;
  }

  const esp_err_t result = m_spi_device.Init(spi::DeviceConfig{
    .pin_cs = SCREEN_PIN_CS,
    .mode = spi::SpiMode::CPOL0_CPHA0,
    .clock_speed_hz = 10'000,

    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .cs_ena_pretrans = 16,
    .cs_ena_posttrans = 16,
    .input_delay_ns = 0,
  });

  if (result != ESP_OK) {
    LOG_E("%s:%d | Error connecting the SPI bus: %s", __FILE__, __LINE__, esp_err_to_name(result));
  }

  return result;
}

void
ScreenDriver::SetupU8g2()
{
  u8g2_Setup_ssd1309_128x64_noname2_f(&m_screen_handle,
                                      U8G2_R0,
                                      u8g2_CallbackSpiByte,
                                      u8g2_CallbackGpioDelay); // init u8g2 structure

  u8g2_InitDisplay(&m_screen_handle); // send init sequence to the display, display is in
                                      // sleep mode after this

  u8g2_SetPowerSave(&m_screen_handle, 0); // wake up display
}

uint8_t
ScreenDriver::u8g2_CallbackSpiByte(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr)
{
  switch (msg) {
    case U8X8_MSG_BYTE_SET_DC:
      gpio_set_level(SCREEN_PIN_DC, arg_int);
      break;

    case U8X8_MSG_BYTE_INIT:
      // SPI should have been initialized in main Init() function before this callback is called.

      if (!spi::Spi2Bus::GetInstance().IsInit()) {
        LOG_E("SPI bus is not initialized. Screen cannot be initialized.");
        return 1;
      }
      break;

    case U8X8_MSG_BYTE_SEND: {
      spi_transaction_t trans_desc;
      trans_desc.addr = 0;
      trans_desc.cmd = 0;
      trans_desc.flags = 0;
      trans_desc.length = 8 * arg_int; // Number of bits NOT number of bytes.
      trans_desc.rxlength = 0;
      trans_desc.tx_buffer = arg_ptr;
      trans_desc.rx_buffer = NULL;

      m_spi_device.TransmitReceiveNonBlocking(trans_desc);

      // ESP_LOGI(TAG, "... Transmitting %d bytes.", arg_int);
      // ESP_ERROR_CHECK(spi_device_transmit(handle_spi, &trans_desc));
      break;
    }

    default:
      break;
  }

  return 0;
}

uint8_t
ScreenDriver::u8g2_CallbackGpioDelay(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr)
{
  switch (msg) {
      // Initialize the GPIO and DELAY HAL functions.  If the pins for DC and
      // RESET have been specified then we define those pins as GPIO outputs.
    case U8X8_MSG_GPIO_AND_DELAY_INIT: {
      uint64_t bitmask = 0;

      bitmask = bitmask | (1ull << SCREEN_PIN_DC);
      bitmask = bitmask | (1ull << SCREEN_PIN_RESET);
      bitmask = bitmask | (1ull << SCREEN_PIN_CS);

      const gpio_config_t gpio_cfg{
        .pin_bit_mask = bitmask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
      };

      gpio_config(&gpio_cfg);
      break;
    }

    // Set the GPIO reset pin to the value passed in through arg_int.
    case U8X8_MSG_GPIO_RESET:
      gpio_set_level(SCREEN_PIN_RESET, arg_int);
      break;

    // Set the GPIO client select pin to the value passed in through arg_int.
    case U8X8_MSG_GPIO_CS:
      gpio_set_level(SCREEN_PIN_CS, arg_int);
      break;

    // Delay for the number of milliseconds passed in through arg_int.
    case U8X8_MSG_DELAY_MILLI:
      vTaskDelay(pdMS_TO_TICKS(arg_int));
      break;
  }

  return 0;
}
