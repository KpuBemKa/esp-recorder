#include "i2s_sampler.hpp"

#include "driver/i2s.h"
#include "esp_err.h"
#include "esp_log.h"
#include "soc/i2s_reg.h"

static const char* TAG = "I2sSampler";

I2sSampler::I2sSampler(i2s_port_t i2s_port,
                               i2s_pin_config_t& i2s_pins,
                               i2s_config_t i2s_config)
  : m_i2sPort(i2s_port)
  , m_i2s_config(i2s_config)
{
  m_i2sPins = i2s_pins;
}

void
I2sSampler::configureI2S()
{
  i2s_set_pin(m_i2sPort, &m_i2sPins);
}

void
I2sSampler::start()
{
  // install and start i2s driver
  i2s_driver_install(m_i2sPort, &m_i2s_config, 0, NULL);
  // set up the I2S configuration from the subclass
  configureI2S();
}

void
I2sSampler::stop()
{
  // stop the i2S driver
  i2s_driver_uninstall(m_i2sPort);
}

int16_t constval = 0;

int
I2sSampler::read(int16_t* samples, int count, bool test)
{
  // read from i2s
  int32_t* raw_samples = (int32_t*)malloc(sizeof(int32_t) * count);
  size_t bytes_read = 0;
  i2s_read(m_i2sPort, raw_samples, sizeof(int32_t) * count, &bytes_read, 100);
  int samples_read = bytes_read / sizeof(int32_t);

  int32_t sample = raw_samples[0];
  sample >>= 14;
  sample = ~sample;
  int16_t out = sample - constval;
  int16_t max = (int16_t)out;
  int16_t min = (int16_t)out;
  for (int i = 0; i < samples_read; i++) {
    sample = raw_samples[i];
    sample >>= 14;
    sample = ~sample;
    // out = sample-11464+(constval*.2);
    out = sample - constval;
    samples[i] = out;
    if (out > max)
      max = out;
    if (out < min)
      min = out;
  }
  if (test)
    constval = constval + ((max + min) / 2 * .05);
  ESP_LOGI(
    TAG, "max=%i\tmin=%i\t (%i)\t%i", max, min, (min + max) / 2, max - min);
  free(raw_samples);
  return samples_read;
}
