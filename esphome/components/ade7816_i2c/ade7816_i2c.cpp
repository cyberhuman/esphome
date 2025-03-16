// This component was developed using knowledge gathered by a number
// of people who reverse-engineered the Shelly 3EM:
//
// @AndreKR on GitHub
// Axel (@Axel830 on GitHub)
// Marko (@goodkiller on GitHub)
// Michaël Piron (@michaelpiron on GitHub)
// Theo Arends (@arendst on GitHub)

#include "ade7816_i2c.h"
#include "esphome/components/ade7816_base/ade7816_registers.h"

namespace esphome {
namespace ade7816_i2c {

static const char *const TAG = "ade7816";

void ADE7816I2C::lock_communication_mode() {
  this->write_u8_register16(ade7816_base::CONFIG2, ade7816_base::CONFIG2_I2C_LOCK);
}

void ADE7816I2C::dump_config() {
  ESP_LOGCONFIG(TAG, "ADE7816_i2c:");
  LOG_I2C_DEVICE(this);
  ade7816_base::ADE7816::dump_config();
}

}  // namespace ade7816_i2c
}  // namespace esphome
