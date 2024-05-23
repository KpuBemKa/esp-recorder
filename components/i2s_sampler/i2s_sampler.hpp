#pragma once

#include <freertos/FreeRTOS.h>
#include <driver/i2s.h>

class I2sSampler
{
private:
    i2s_pin_config_t m_i2sPins;

protected:
    i2s_port_t m_i2sPort = I2S_NUM_0;
    i2s_config_t m_i2s_config;
    void configureI2S();

public:
    I2sSampler(i2s_port_t i2s_port, i2s_pin_config_t &i2s_pins, i2s_config_t i2s_config);
    void start();
    void stop();
    int read(int16_t *samples, int count,bool test=false);
    int sample_rate()
    {
        return m_i2s_config.sample_rate;
    }
};
