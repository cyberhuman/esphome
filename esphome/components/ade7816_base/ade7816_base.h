#pragma once

#include "esphome/components/ade78xx_base/ade78xx_base.h"

namespace esphome {
namespace ade7816_base {

class ADE7816 : public ade78xx_base::ADE78xx {
 protected:
  bool check_reset_status_good() override;
  void clear_status_registers() override;
  void init_device_registers() override;
  void flush_write_queue() override;
  void enable_write_protection() override;
  void enable_dsp() override;

  void software_reset_device() override;
};

}  // namespace ade7816_base
}  // namespace esphome
