#pragma once

#include <span>

#include "spi2_bus.hpp"

namespace spi {

class SpiDevice
{
public:
  SpiDevice();

  esp_err_t Init(const DeviceConfig& dev_config);
  esp_err_t DeInit();

  bool IsInit();
  bool IsBusInit();

  /// @brief Transmit and receive data in blocking mode.
  /// Waits for the transaction to finish without delaying the task by using a spinlock.
  /// Is faster that non-blocking mode, but wastes CPU time
  /// @param transaction_info a struct with information about the transaction
  /// @return `esp_err_t`
  esp_err_t TransmitReceiveBlocking(spi_transaction_t& trans_desc);

  /// @brief Transmit and receive data in non-blocking mode.
  /// Waits for the transaction to finish by delaying the task and using ISR callbacks.
  /// Is slower than blocking mode, but does not waste CPU time as much
  /// @param transaction_info a struct with information about the transaction
  /// @return `esp_err_t`
  esp_err_t TransmitReceiveNonBlocking(spi_transaction_t& trans_desc);

  /// @brief Returns internal `spi_device_handle_t`.
  /// May be used to pass directly to ESP-IDF API functions
  /// @return internal `spi_device_handle_t`
  spi_device_handle_t GetDeviceHandle();

private:
  Spi2Bus& m_spi_bus;
  spi_device_handle_t m_device_handle;

  /// Device id is the respective number of device's Chip Select GPIO pin
  /// Used to show about which device an error is about,
  /// and also can be used to check if a device is initialized or not
  gpio_num_t m_device_id = gpio_num_t::GPIO_NUM_NC;
};

} // namespace spi
