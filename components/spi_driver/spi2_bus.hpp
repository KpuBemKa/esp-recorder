#pragma once

#include "base/spi_bus.hpp"

namespace spi {

class Spi2Bus : public SpiBus
{
public:
  Spi2Bus(Spi2Bus& other) = delete;
  void operator=(Spi2Bus& other) = delete;

  static Spi2Bus& GetInstance()
  {
    static Spi2Bus instance;
    return instance;
  }

  ~Spi2Bus()
  {
    SpiBus::DeInitBus(SPI2_HOST);
  }

  /// @brief Init the SPI2 bus in 3 line mode
  /// @param pin_clk Clock pin
  /// @param pin_mosi Master Out Slave In pin
  /// @param pin_miso Master In Slave Out pin
  /// @return `ESP_OK` if successful, `esp_err_t` otherwise
  esp_err_t InitBus(const gpio_num_t pin_clk, const gpio_num_t pin_mosi, const gpio_num_t pin_miso)
  {
    return SpiBus::InitBus(SPI2_HOST, pin_clk, pin_mosi, pin_miso);
  }

  /// @brief De-initialize SPI2 bus
  /// @return
  esp_err_t DeInitBus() { return SpiBus::DeInitBus(SPI2_HOST); }

  /// @brief Add a device to the bus
  /// @param device_config device configuration
  /// @return `spi_device_handle_t` in case of success, `esp_err_t` otherwise
  std::expected<spi_device_handle_t, esp_err_t> AddDevice(const DeviceConfig& device_config)
  {
    return SpiBus::AddDevice(SPI2_HOST, device_config);
  }

  /// @brief Remove a device from the bus
  /// @param device_handle `spi_device_handle_t` of respective device
  /// @return `esp_err_t`
  esp_err_t RemoveDevice(spi_device_handle_t device_handle)
  {
    return SpiBus::RemoveDevice(device_handle);
  }

  bool IsInit() { return SpiBus::IsInit(); }

private:
  Spi2Bus() {}
};

} // namespace spi
