#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/ade78xx_base/ade78xx_spi.h"
#include "esphome/components/ade7816_base/ade7816_base.h"

namespace esphome {
namespace ade7816_spi {

class ADE7816SPI : public ade78xx_base::ADE78xxSPI<ade7816_base::ADE7816, ADE7816SPI>,
                   public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_HIGH, spi::CLOCK_PHASE_LEADING,
                                         spi::DATA_RATE_1MHZ> {
 public:
  void setup() override;

  void dump_config() override;

 protected:
  void lock_communication_mode() override;
};

}  // namespace ade7816_spi
}  // namespace esphome
