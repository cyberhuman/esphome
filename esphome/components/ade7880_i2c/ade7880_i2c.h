#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/ade78xx_base/ade78xx_i2c.h"
#include "esphome/components/ade7880_base/ade7880_base.h"

namespace esphome {
namespace ade7880_i2c {

class ADE7880I2C : public ade78xx_base::ADE78xxI2C<ade7880_base::ADE7880, ADE7880I2C>, public i2c::I2CDevice {
 public:
  void dump_config() override;

 protected:
  void lock_communication_mode() override;
};

}  // namespace ade7880_i2c
}  // namespace esphome
