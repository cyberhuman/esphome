#pragma once

// This component was developed using knowledge gathered by a number
// of people who reverse-engineered the Shelly 3EM:
//
// @AndreKR on GitHub
// Axel (@Axel830 on GitHub)
// Marko (@goodkiller on GitHub)
// Michaël Piron (@michaelpiron on GitHub)
// Theo Arends (@arendst on GitHub)

#include "esphome/components/ade78xx_base/ade78xx_base.h"

namespace esphome {
namespace ade7880_base {

class ADE7880 : public ade78xx_base::ADE78xx {
 protected:
  bool check_reset_status_good() override;
  void clear_status_registers() override;
  void init_device_registers() override;
  void flush_write_queue() override;
  void enable_write_protection() override;
  void enable_dsp() override;

  void software_reset_device() override;
};

}  // namespace ade7880_base
}  // namespace esphome
