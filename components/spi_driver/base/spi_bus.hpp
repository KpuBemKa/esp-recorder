#pragma once

#include <cstdint>
#include <expected>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "hal/gpio_types.h"

namespace spi {

enum class SpiMode
{
  CPOL0_CPHA0 = 0,
  CPOL0_CPHA1 = 1,
  CPOL1_CPHA0 = 2,
  CPOL1_CPHA1 = 3,
};

struct DeviceConfig
{
  gpio_num_t pin_cs;  ///< CS GPIO pin for this device, or -1 if not used
  SpiMode mode;       ///< SPI mode, representing a pair of (CPOL, CPHA) configuration
  int clock_speed_hz; ///< SPI clock speed in Hz. Derived from `clock_source`.

  uint8_t command_bits; ///< Default amount of bits in command phase (0-16), used when
                        ///< ``SPI_TRANS_VARIABLE_CMD`` is not used, otherwise ignored.
  uint8_t address_bits; ///< Default amount of bits in address phase (0-64), used when
                        ///< ``SPI_TRANS_VARIABLE_ADDR`` is not used, otherwise ignored.
  uint8_t dummy_bits;   ///< Amount of dummy bits to insert between address and data phase

  uint16_t cs_ena_pretrans; ///< Amount of SPI bit-cycles the cs should be activated before the
                            ///< transmission (0-16). This only works on half-duplex transactions.
  uint8_t cs_ena_posttrans; ///< Amount of SPI bit-cycles the cs should stay active after the
                            ///< transmission (0-16)
  int input_delay_ns; /**< Maximum data valid time of slave. The time required between SCLK and MISO
                        valid, including the possible clock delay from slave to master.
                        The driver uses this value to give an extra delay before
                        the MISO is ready on the line. Leave at 0 unless you know you need a delay.
                        For better timing performance at high frequency (over 8MHz),
                        it's suggested to have the right value.
                        */
};

class SpiBus
{
protected:
  /// @brief Init the SPI bus in 3 line mode
  /// @param bus_id SPI channel to initialize
  /// @param pin_clk Clock pin
  /// @param pin_mosi Master Out Slave In pin
  /// @param pin_miso Master In Slave Out pin
  /// @return `ESP_OK` if successful, `esp_err_t` otherwise
  esp_err_t InitBus(const spi_host_device_t bus_id,
                    const gpio_num_t pin_clk,
                    const gpio_num_t pin_mosi,
                    const gpio_num_t pin_miso);
  esp_err_t DeInitBus(const spi_host_device_t bus_id);

  std::expected<spi_device_handle_t, esp_err_t> AddDevice(const spi_host_device_t bus_id,
                                                          const DeviceConfig& device_config);
  esp_err_t RemoveDevice(spi_device_handle_t device_handle);

  bool IsInit() { return m_is_init; }

  /// @brief Use after adding a device without using this API
  // void DeviceAdded() { ++m_device_count; }
  /// @brief Use after removing a device without using this API
  // void DeviceRemoved() { --m_device_count; }

private:
  bool m_is_init = false;
  // std::size_t m_device_count = 0;
};

} // namespace spi
