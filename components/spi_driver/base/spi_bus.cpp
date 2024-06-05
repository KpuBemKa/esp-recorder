#include "spi_bus.hpp"

namespace spi {

esp_err_t
SpiBus::InitBus(const spi_host_device_t bus_id,
                const gpio_num_t pin_clk,
                const gpio_num_t pin_mosi,
                const gpio_num_t pin_miso)
{
  const spi_bus_config_t bus_config{
    .mosi_io_num = pin_mosi,
    .miso_io_num = pin_miso,
    .sclk_io_num = pin_clk,
    .quadwp_io_num = gpio_num_t::GPIO_NUM_NC,
    .quadhd_io_num = gpio_num_t::GPIO_NUM_NC,
    .data4_io_num = gpio_num_t::GPIO_NUM_NC,
    .data5_io_num = gpio_num_t::GPIO_NUM_NC,
    .data6_io_num = gpio_num_t::GPIO_NUM_NC,
    .data7_io_num = gpio_num_t::GPIO_NUM_NC,
    .max_transfer_sz = 0,
    .flags = SPICOMMON_BUSFLAG_MASTER,
    .isr_cpu_id = esp_intr_cpu_affinity_t::ESP_INTR_CPU_AFFINITY_AUTO,
    .intr_flags = 0,
  };

  const esp_err_t result = spi_bus_initialize(bus_id, &bus_config, SPI_DMA_CH_AUTO);

  if (result == ESP_OK) {
    m_is_init = true;
  }

  return result;
}

esp_err_t
SpiBus::DeInitBus(const spi_host_device_t bus_id)
{
  const esp_err_t result = spi_bus_free(bus_id);

  if (result == ESP_OK) {
    m_is_init = false;
  }

  return result;
}

std::expected<spi_device_handle_t, esp_err_t>
SpiBus::AddDevice(const spi_host_device_t bus_id, const DeviceConfig& device_config)
{
  const spi_device_interface_config_t dev_config{
    .command_bits = device_config.command_bits,
    .address_bits = device_config.address_bits,
    .dummy_bits = device_config.dummy_bits,
    .mode = static_cast<uint8_t>(device_config.mode),

    .clock_source = spi_clock_source_t::SPI_CLK_SRC_DEFAULT,
    .duty_cycle_pos = 128, ///< 50%/50% duty
    .cs_ena_pretrans = device_config.cs_ena_pretrans,
    .cs_ena_posttrans = device_config.cs_ena_posttrans,
    .clock_speed_hz = device_config.clock_speed_hz,
    .input_delay_ns = device_config.input_delay_ns,
    .spics_io_num = device_config.pin_cs,
    .flags = 0,
    .queue_size = 200,
    .pre_cb = nullptr,
    .post_cb = nullptr,
  };

  spi_device_handle_t device_handle;
  const esp_err_t result = spi_bus_add_device(bus_id, &dev_config, &device_handle);

  if (result == ESP_OK) {
    return device_handle;
  } else {
    return std::unexpected(result);
  }
}

esp_err_t
SpiBus::RemoveDevice(spi_device_handle_t device_handle)
{
  return spi_bus_remove_device(device_handle);
}

} // namespace spi