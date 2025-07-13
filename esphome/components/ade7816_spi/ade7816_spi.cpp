#include "ade7816_spi.h"
#include "esphome/core/log.h"
#include "esphome/components/ade7816_base/ade7816_registers.h"

namespace esphome {
namespace ade7816_spi {

static const char *const TAG = "ade7816";

void ADE7816SPI::setup() {
  this->spi_setup();
  ade7816_base::ADE7816::setup();
}

void ADE7816SPI::dump_config() {
  ESP_LOGCONFIG(TAG, "ADE7816_spi:");
  LOG_PIN("  CS Pin: ", this->cs_);
  ade7816_base::ADE7816::dump_config();
}

void ADE7816SPI::lock_communication_mode() {
  for (int i = 0; i < 3; i++) {
    this->write_u8_register16(ade7816_base::RESERVED_EBFF, 0x00);
  }
  this->write_u8_register16(ade7816_base::CONFIG2, ade7816_base::CONFIG2_SPI_LOCK);
}

}  // namespace ade7816_spi
}  // namespace esphome
