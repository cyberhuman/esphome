// This component was developed using knowledge gathered by a number
// of people who reverse-engineered the Shelly 3EM:
//
// @AndreKR on GitHub
// Axel (@Axel830 on GitHub)
// Marko (@goodkiller on GitHub)
// Michaël Piron (@michaelpiron on GitHub)
// Theo Arends (@arendst on GitHub)

#include "ade7880_i2c.h"
#include "esphome/components/ade7880_base/ade7880_registers.h"

namespace esphome {
namespace ade7880_i2c {

static const char *const TAG = "ade7880";

void ADE7880I2C::lock_communication_mode() {
  this->write_u8_register16(ade7880_base::CONFIG2, ade7880_base::CONFIG2_I2C_LOCK);
}

void ADE7880I2C::dump_config() {
  ESP_LOGCONFIG(TAG, "ADE7880_i2c:");
  LOG_I2C_DEVICE(this);
  ade7880_base::ADE7880::dump_config();
}

}  // namespace ade7880_i2c
}  // namespace esphome
