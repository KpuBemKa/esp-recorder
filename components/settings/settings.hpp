#pragma once

#include <cstdint>
#include <string_view>

#include "hal/gpio_types.h"

constexpr std::size_t MIC_SAMPLE_RATE = 16'000; // Hz

// I2S microphone pins
constexpr gpio_num_t I2S_MIC_SERIAL_CLOCK = gpio_num_t::GPIO_NUM_23;
constexpr gpio_num_t I2S_MIC_LEFT_RIGHT_CLOCK = gpio_num_t::GPIO_NUM_22;
constexpr gpio_num_t I2S_MIC_SERIAL_DATA = gpio_num_t::GPIO_NUM_21;

// Record button pin
constexpr gpio_num_t BUTTON_PIN = gpio_num_t::GPIO_NUM_15;

// SD card pins
constexpr gpio_num_t SD_PIN_MISO = gpio_num_t::GPIO_NUM_6;
constexpr gpio_num_t SD_PIN_CLK = gpio_num_t::GPIO_NUM_5;
constexpr gpio_num_t SD_PIN_MOSI = gpio_num_t::GPIO_NUM_4;
constexpr gpio_num_t SD_PIN_CS = gpio_num_t::GPIO_NUM_1;

constexpr std::string_view VFS_MOUNT_POINT = "/sdcard";

#define DEBUG_SD 1
#define DEBUG_MIC 1
#define DEBUG_WAV 1
#define DEBUG_COM 1