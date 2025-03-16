// This component was developed using knowledge gathered by a number
// of people who reverse-engineered the Shelly 3EM:
//
// @AndreKR on GitHub
// Axel (@Axel830 on GitHub)
// Marko (@goodkiller on GitHub)
// Michaël Piron (@michaelpiron on GitHub)
// Theo Arends (@arendst on GitHub)

#include "ade7880_base.h"
#include "ade7880_registers.h"
#include "esphome/core/log.h"

#include <cinttypes>

namespace esphome {
namespace ade7880_base {

static const char *const TAG = "ade7880";

bool ADE7880::check_reset_status_good() {
  auto status1 = read_u32_register16(STATUS1);
  if ((status1 & ~STATUS1_RSTDONE) != 0) {
    // not safe to proceed, must initiate reset
    ESP_LOGD(TAG, "IRQ1 asserted for !RSTDONE, resetting device");
    return false;
  }
  return ((status1 & STATUS1_RSTDONE) == STATUS1_RSTDONE);
}

void ADE7880::clear_status_registers() {
  this->write_u32_register16(STATUS0, 0xFFFF);
  this->write_u32_register16(STATUS1, 0xFFFF);
}

void ADE7880::init_device_registers() {
  this->write_u16_register16(GAIN, 0);
  if (this->frequency_ > 55) {
    this->write_u16_register16(COMPMODE, COMPMODE_DEFAULT | COMPMODE_SELFREQ);
  }
}

void ADE7880::flush_write_queue() {
  // write three default values to data memory RAM to flush the I2C write queue
  this->write_s32_register16(VLEVEL, 0);
  this->write_s32_register16(VLEVEL, 0);
  this->write_s32_register16(VLEVEL, 0);
}

void ADE7880::enable_write_protection() {
  this->write_u8_register16(DSPWP_SEL, DSPWP_SEL_SET);
  this->write_u8_register16(DSPWP_SET, DSPWP_SET_RO);
}

void ADE7880::enable_dsp() { this->write_u16_register16(RUN, RUN_ENABLE); }

void ADE7880::software_reset_device() { this->write_u16_register16(CONFIG, CONFIG_SWRST); }

}  // namespace ade7880_base
}  // namespace esphome
