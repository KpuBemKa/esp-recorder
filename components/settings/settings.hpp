#pragma once

#include <cstdint>
#include <string_view>

#include "hal/gpio_types.h"

constexpr std::size_t MIC_SAMPLE_RATE = 16'000; // Hz
// constexpr std::size_t MIC_SAMPLE_RATE = 32'000; // Hz
// constexpr std::size_t MIC_SAMPLE_RATE = 44'100; // Hz

// I2S microphone pins
constexpr gpio_num_t I2S_MIC_SERIAL_DATA = gpio_num_t::GPIO_NUM_23;
constexpr gpio_num_t I2S_MIC_SERIAL_CLOCK = gpio_num_t::GPIO_NUM_22;
constexpr gpio_num_t I2S_MIC_WORD_SELECT = gpio_num_t::GPIO_NUM_21;
// constexpr gpio_num_t I2S_MIC_SERIAL_DATA = gpio_num_t::GPIO_NUM_22;
// constexpr gpio_num_t I2S_MIC_LEFT_RIGHT_CLOCK = gpio_num_t::GPIO_NUM_21;

// Record button pin
constexpr gpio_num_t BUTTON_PIN = gpio_num_t::GPIO_NUM_15;

// Internal addressable RGB LED
constexpr gpio_num_t ARGB_LED_PIN = gpio_num_t::GPIO_NUM_8;

constexpr gpio_num_t SPI_PIN_CLK = gpio_num_t::GPIO_NUM_6;
constexpr gpio_num_t SPI_PIN_MOSI = gpio_num_t::GPIO_NUM_7;
constexpr gpio_num_t SPI_PIN_MISO = gpio_num_t::GPIO_NUM_2;

// SD card pins
constexpr gpio_num_t SD_PIN_CS = gpio_num_t::GPIO_NUM_19;

// Screen pins
constexpr gpio_num_t SCREEN_PIN_RESET = gpio_num_t::GPIO_NUM_10;
constexpr gpio_num_t SCREEN_PIN_DC = gpio_num_t::GPIO_NUM_11;
constexpr gpio_num_t SCREEN_PIN_CS = gpio_num_t::GPIO_NUM_18;

constexpr std::string_view VFS_MOUNT_POINT = "/storage";

#define DEBUG_SD 1
#define DEBUG_MIC 1
#define DEBUG_WAV 1
#define DEBUG_COM 1
#define DEBUG_SCREEN 1
#define DEBUG_SPI 1