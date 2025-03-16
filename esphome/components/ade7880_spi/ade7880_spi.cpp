#include "ade7880_spi.h"
#include "esphome/core/log.h"
#include "esphome/components/ade7880_base/ade7880_registers.h"

namespace esphome {
namespace ade7880_spi {

static const char *const TAG = "ade7880";

void ADE7880SPI::setup() {
  this->spi_setup();
  ade7880_base::ADE7880::setup();
}

void ADE7880SPI::dump_config() {
  ESP_LOGCONFIG(TAG, "ADE7880_spi:");
  LOG_PIN("  CS Pin: ", this->cs_);
  ade7880_base::ADE7880::dump_config();
}

void ADE7880SPI::lock_communication_mode() {
  for (int i = 0; i < 3; i++) {
    this->write_u8_register16(ade7880_base::RESERVED_EBFF, 0x00);
  }
  this->write_u8_register16(ade7880_base::CONFIG2, ade7880_base::CONFIG2_SPI_LOCK);
}

}  // namespace ade7880_spi
}  // namespace esphome
