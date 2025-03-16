// This component was developed using knowledge gathered by a number
// of people who reverse-engineered the Shelly 3EM:
//
// @AndreKR on GitHub
// Axel (@Axel830 on GitHub)
// Marko (@goodkiller on GitHub)
// Michaël Piron (@michaelpiron on GitHub)
// Theo Arends (@arendst on GitHub)

#include "ade78xx_base.h"
#include "esphome/core/log.h"
#include "esphome/core/optional.h"

#include <cinttypes>

namespace esphome {
namespace ade78xx_base {

static const char *const TAG = "ade78xx";

void IRAM_ATTR ADE78xxStore::gpio_intr(ADE78xxStore *arg) { arg->reset_done = true; }

void ADE78xx::setup() {
  if (this->irq0_pin_ != nullptr) {
    this->irq0_pin_->setup();
  }
  this->irq1_pin_->setup();
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
  }
  this->store_.irq1_pin = this->irq1_pin_->to_isr();
  this->irq1_pin_->attach_interrupt(ADE78xxStore::gpio_intr, &this->store_, gpio::INTERRUPT_FALLING_EDGE);

  // if IRQ1 is already asserted, the cause must be determined
  if (this->irq1_pin_->digital_read() == 0) {
    ESP_LOGD(TAG, "IRQ1 found asserted during setup()");
    if (this->check_reset_status_good()) {
      // safe to proceed, device has just completed reset cycle
      ESP_LOGD(TAG, "Acknowledging RSTDONE");
      this->clear_status_registers();
      this->init_device_();
      return;
    }
  }

  this->reset_device_();
}

void ADE78xx::loop() {
  // check for completion of a reset cycle
  if (!this->store_.reset_done) {
    return;
  }

  ESP_LOGD(TAG, "Acknowledging RSTDONE");
  this->clear_status_registers();
  this->init_device_();
  this->store_.reset_done = false;
  this->store_.reset_pending = false;
}

template<typename T, typename F>
void ADE78xx::update_sensor_from_register_(sensor::Sensor *sensor, T (ADE78xx::*read_register)(uint16_t),
                                           const optional<uint16_t> &a_register, F &&f) {
  if (!a_register || sensor == nullptr) {
    return;
  }

  float val = (this->*read_register)(*a_register);
  sensor->publish_state(f(val));
}

void ADE78xx::update() {
  if (this->store_.reset_pending) {
    return;
  }

  auto start = millis();

  for (auto *chan : this->channels_) {
    this->update_sensor_from_register_(chan->current, &ADE78xx::read_s24zp_register16_, chan->irms_,
                                       [](float val) { return val / 100000.0f; });
    this->update_sensor_from_register_(chan->voltage, &ADE78xx::read_s24zp_register16_, chan->vrms_,
                                       [](float val) { return val / 10000.0f; });
    this->update_sensor_from_register_(chan->active_power, &ADE78xx::read_s24zp_register16_, chan->watt_,
                                       [](float val) { return val / 100.0f; });
    this->update_sensor_from_register_(chan->apparent_power, &ADE78xx::read_s24zp_register16_, chan->va_,
                                       [](float val) { return val / 100.0f; });
    this->update_sensor_from_register_(chan->power_factor, &ADE78xx::read_s16_register16, chan->pf_,
                                       [](float val) { return std::abs(val / -327.68f); });
    this->update_sensor_from_register_(
        chan->forward_active_energy, &ADE78xx::read_s32_register16, chan->fwatthr_,
        [&chan](float val) { return chan->forward_active_energy_total += val / 14400.0f; });
    this->update_sensor_from_register_(
        chan->reverse_active_energy, &ADE78xx::read_s32_register16, chan->fvarhr_,
        [&chan](float val) { return chan->reverse_active_energy_total += val / 14400.0f; });
  }

  ESP_LOGD(TAG, "update took %" PRIu32 " ms", millis() - start);
}

void ADE78xx::dump_config() {
  LOG_PIN("  IRQ0  Pin: ", this->irq0_pin_);
  LOG_PIN("  IRQ1  Pin: ", this->irq1_pin_);
  LOG_PIN("  RESET Pin: ", this->reset_pin_);
  ESP_LOGCONFIG(TAG, "  Frequency: %.0f Hz", this->frequency_);

  for (auto *chan : this->channels_) {
    ESP_LOGCONFIG(TAG, "  %s:", chan->name_.c_str());
    LOG_SENSOR("    ", "Current", chan->current);
    LOG_SENSOR("    ", "Voltage", chan->voltage);
    LOG_SENSOR("    ", "Active Power", chan->active_power);
    LOG_SENSOR("    ", "Apparent Power", chan->apparent_power);
    LOG_SENSOR("    ", "Power Factor", chan->power_factor);
    LOG_SENSOR("    ", "Forward Active Energy", chan->forward_active_energy);
    LOG_SENSOR("    ", "Reverse Active Energy", chan->reverse_active_energy);
    ESP_LOGCONFIG(TAG, "    Calibration:");
    if (chan->current_gain_calibration)
      ESP_LOGCONFIG(TAG, "     Current: %" PRId32, *chan->current_gain_calibration);
    if (chan->voltage_gain_calibration)
      ESP_LOGCONFIG(TAG, "     Voltage: %" PRId32, *chan->voltage_gain_calibration);
    if (chan->power_gain_calibration)
      ESP_LOGCONFIG(TAG, "     Power: %" PRId32, *chan->power_gain_calibration);
    if (chan->phase_angle_calibration)
      ESP_LOGCONFIG(TAG, "     Phase Angle: %u", *chan->phase_angle_calibration);
  }

  LOG_UPDATE_INTERVAL(this);
}

void ADE78xx::calibrate_s10zp_reading_(const optional<uint16_t> &a_register, const optional<int16_t> &calibration) {
  if (!a_register || !calibration) {
    return;
  }

  this->write_s10zp_register16_(*a_register, *calibration);
}

void ADE78xx::calibrate_s24zpse_reading_(const optional<uint16_t> &a_register, const optional<int32_t> &calibration) {
  if (!a_register || !calibration) {
    return;
  }

  this->write_s24zpse_register16_(*a_register, *calibration);
}

void ADE78xx::init_device_() {
  this->lock_communication_mode();

  this->init_device_registers();

  for (auto *chan : this->channels_) {
    this->calibrate_s24zpse_reading_(chan->igain_, chan->current_gain_calibration);
    this->calibrate_s24zpse_reading_(chan->vgain_, chan->voltage_gain_calibration);
    this->calibrate_s24zpse_reading_(chan->pgain_, chan->power_gain_calibration);
    this->calibrate_s24zpse_reading_(chan->phcal_24bit_, chan->phase_angle_calibration);
    this->calibrate_s10zp_reading_(chan->phcal_10bit_, chan->phase_angle_calibration);
  }

  this->flush_write_queue();

  this->enable_write_protection();
  this->enable_dsp();
}

void ADE78xx::reset_device_() {
  if (this->reset_pin_ != nullptr) {
    ESP_LOGD(TAG, "Reset device using RESET pin");
    this->reset_pin_->digital_write(false);
    delay(1);
    this->reset_pin_->digital_write(true);
  } else {
    ESP_LOGD(TAG, "Reset device using SWRST command");
    this->software_reset_device();
  }
  this->store_.reset_pending = true;
}

// adapted from https://stackoverflow.com/a/55912127/1886371
template<size_t Bits, typename T> inline T sign_extend(const T &v) noexcept {
  using S = struct { signed Val : Bits; };
  return reinterpret_cast<const S *>(&v)->Val;
}

// Register types
// unsigned 8-bit (uint8_t)
// signed 10-bit - 16-bit ZP on wire (int16_t, needs sign extension)
// unsigned 16-bit (uint16_t)
// unsigned 20-bit - 32-bit ZP on wire (uint32_t)
// signed 24-bit - 32-bit ZPSE on wire (int32_t, needs sign extension)
// signed 24-bit - 32-bit ZP on wire (int32_t, needs sign extension)
// signed 24-bit - 32-bit SE on wire (int32_t)
// signed 28-bit - 32-bit ZP on wire (int32_t, needs sign extension)
// unsigned 32-bit (uint32_t)
// signed 32-bit (int32_t)

int32_t ADE78xx::read_s24zp_register16_(uint16_t a_register) {
  // s24zp means 24 bit signed value in the lower 24 bits of a 32-bit register
  int32_t in = this->read_s32_register16(a_register);
  return sign_extend<24>(in);
}

void ADE78xx::write_s10zp_register16_(uint16_t a_register, int16_t value) {
  this->write_s16_register16(a_register, value & 0x03FF);
}

void ADE78xx::write_s24zpse_register16_(uint16_t a_register, int32_t value) {
  // s24zpse means a 24-bit signed value, sign-extended to 28 bits, in the lower 28 bits of a 32-bit register
  this->write_s32_register16(a_register, value & 0x0FFFFFFF);
}

}  // namespace ade78xx_base
}  // namespace esphome
