#include "ade7816_base.h"
#include "ade7816_registers.h"
#include "esphome/core/log.h"

#include <cinttypes>

namespace esphome {
namespace ade7816_base {

static const char *const TAG = "ade7816";

bool ADE7816::check_reset_status_good() {
  auto status1 = read_u32_register16(STATUS1);
  if ((status1 & ~STATUS1_RSTDONE) != 0) {
    // not safe to proceed, must initiate reset
    ESP_LOGD(TAG, "IRQ1 asserted for !RSTDONE, resetting device");
    return false;
  }
  return ((status1 & STATUS1_RSTDONE) == STATUS1_RSTDONE);
}

void ADE7816::clear_status_registers() {
  this->write_u32_register16(STATUS0, 0x0003FFFF);
  this->write_u32_register16(STATUS1, 0x01FFFFFF);
}

void ADE7816::init_device_registers() {
  this->read_u32_register16(CHECKSUM);
  this->write_u32_register16(WTHR1, 0x000002);
  this->write_u32_register16(WTHR0, 0x000000);
  this->write_u32_register16(VARTHR1, 0x000002);
  this->write_u32_register16(VARTHR0, 0x000000);
  this->write_u16_register16(GAIN, 0);
}

void ADE7816::flush_write_queue() {
  // write three default values to data memory RAM to flush the I2C write queue
  this->write_s32_register16(HPFDIS, 0);
  this->write_s32_register16(HPFDIS, 0);
  this->write_s32_register16(HPFDIS, 0);
}

void ADE7816::enable_write_protection() {
  this->write_u8_register16(DSPWP_SEL, DSPWP_SEL_SET);
  this->write_u8_register16(DSPWP_SET, DSPWP_SET_RO);
}

void ADE7816::enable_dsp() {
  this->write_u16_register16(RUN, RUN_ENABLE);
  this->read_u16_register16(RUN);
}

void ADE7816::software_reset_device() { this->write_u16_register16(CONFIG, CONFIG_SWRST); }

}  // namespace ade7816_base
}  // namespace esphome
