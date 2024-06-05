#include "spi_device.hpp"

#include <esp_log.h>

#include "settings.hpp"

const char TAG[] = "SPI";

#if DEBUG_SPI
#define LOG_I(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define LOG_I(...)
#endif

#define LOG_E(...) ESP_LOGE(TAG, __VA_ARGS__)
#define LOG_W(...) ESP_LOGW(TAG, __VA_ARGS__)

namespace spi {

SpiDevice::SpiDevice()
  : m_spi_bus(spi::Spi2Bus::GetInstance())
{
}

esp_err_t
SpiDevice::Init(const DeviceConfig& dev_config)
{
  if (IsInit()) {
    LOG_W("Spi device with CS pin `%d` is already initialized.", m_device_id);
    return ESP_ERR_INVALID_STATE;
  }

  std::expected<spi_device_handle_t, esp_err_t> result = m_spi_bus.AddDevice(dev_config);

  if (!result.has_value()) {
    LOG_E("%s:%d | Error adding device with CS pin `%d` to the bus: %s",
          __FILE__,
          __LINE__,
          dev_config.pin_cs,
          esp_err_to_name(result.error()));
    return result.error();
  }

  m_device_handle = result.value();
  m_device_id = dev_config.pin_cs;

  return ESP_OK;
}

esp_err_t
SpiDevice::DeInit()
{
  if (!IsInit()) {
    LOG_W("Spi device with CS pin `%d` is already de-initialized.", m_device_id);
    return ESP_ERR_INVALID_STATE;
  }

  const esp_err_t result = m_spi_bus.RemoveDevice(m_device_handle);
  if (result != ESP_OK) {
    LOG_E("%s:%d | Error removing device with CS pin `%d` from the bus: %s",
          __FILE__,
          __LINE__,
          m_device_id,
          esp_err_to_name(result));
    return result;
  }

  m_device_handle = nullptr;
  m_device_id = gpio_num_t::GPIO_NUM_NC;

  return ESP_OK;
}

bool
SpiDevice::IsInit()
{
  // if device id is the invalid GPIO pin, device is not initialized
  return m_device_id != gpio_num_t::GPIO_NUM_NC;
}

bool
SpiDevice::IsBusInit()
{
  return m_spi_bus.IsInit();
}

esp_err_t
SpiDevice::TransmitReceiveBlocking(spi_transaction_t& trans_desc)
{
  const esp_err_t result = spi_device_polling_transmit(m_device_handle, &trans_desc);

  if (result != ESP_OK) {
    LOG_E("%s:%d | Device `%d` transmit error: %s",
          __FILE__,
          __LINE__,
          m_device_id,
          esp_err_to_name(result));
  }

  return result;
}

esp_err_t
SpiDevice::TransmitReceiveNonBlocking(spi_transaction_t& trans_desc)
{
  const esp_err_t result = spi_device_transmit(m_device_handle, &trans_desc);

  if (result != ESP_OK) {
    LOG_E("%s:%d | Device `%d` transmit error: %s",
          __FILE__,
          __LINE__,
          m_device_id,
          esp_err_to_name(result));
  }

  return result;
}

spi_device_handle_t
SpiDevice::GetDeviceHandle()
{
  return m_device_handle;
}

} // namespace spi