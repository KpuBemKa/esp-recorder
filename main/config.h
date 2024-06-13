// #include <driver/i2s.h>
// #include <freertos/FreeRTOS.h>

// save to SPIFFS instead of SD Card?
// #define USE_SPIFFS 0

// #define SAMPLE_RATE 16'000 // Hz

// are you using an I2S microphone - comment this out if you want to use an analog mic and ADC input
// #define USE_I2S_MIC_INPUT

// I2S Microphone Settings
// Which channel is the I2S microphone on? I2S_CHANNEL_FMT_ONLY_LEFT or I2S_CHANNEL_FMT_ONLY_RIGHT
// Generally they will default to LEFT - but you may need to attach the L/R pin to GND
// constexpr i2s_channel_fmt_t I2S_MIC_CHANNEL = I2S_CHANNEL_FMT_ONLY_LEFT;
// #define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_RIGHT
// #define I2S_MIC_SERIAL_CLOCK 23
// #define I2S_MIC_LEFT_RIGHT_CLOCK 22
// #define I2S_MIC_SERIAL_DATA 21

// record button
// #define GPIO_BUTTON GPIO_NUM_15

// // sdcard
// #define PIN_NUM_MISO GPIO_NUM_6
// #define PIN_NUM_CLK GPIO_NUM_5
// #define PIN_NUM_MOSI GPIO_NUM_4
// #define PIN_NUM_CS GPIO_NUM_1

  // i2s config for reading from of I2S
  // extern i2s_config_t i2s_mic_Config;
  // // i2s microphone pins
  // extern i2s_pin_config_t i2s_mic_pins;
