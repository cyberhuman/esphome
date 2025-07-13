#pragma once

#include "esphome/core/helpers.h"
#include "ade78xx_base.h"

namespace esphome {
namespace ade78xx_base {

template<typename Base, typename T> class ADE78xxSPI : public Base {
 public:
  T &derived() { return static_cast<T &>(*this); }
  const T &derived() const { return static_cast<const T &>(*this); }

 protected:
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

  uint8_t read_u8_register16(uint16_t a_register) override {
    uint8_t in;
    derived().enable();
    derived().write_byte(0x01);
    derived().write_byte16(a_register);
    in = derived().read_byte();
    derived().disable();
    return in;
  }

  int16_t read_s16_register16(uint16_t a_register) override {
    return static_cast<int16_t>(this->read_u16_register16(a_register));
  }

  uint16_t read_u16_register16(uint16_t a_register) override {
    uint16_t in;

    derived().enable();
    derived().write_byte(0x01);
    derived().write_byte16(a_register);
    derived().read_array(reinterpret_cast<uint8_t *>(&in), sizeof(in));
    derived().disable();
    return convert_big_endian(in);
  }

  int32_t read_s32_register16(uint16_t a_register) override {
    return static_cast<int32_t>(this->read_u32_register16(a_register));
  }

  uint32_t read_u32_register16(uint16_t a_register) override {
    uint32_t in;

    derived().enable();
    derived().write_byte(0x01);
    derived().write_byte16(a_register);
    derived().read_array(reinterpret_cast<uint8_t *>(&in), sizeof(in));
    derived().disable();
    return convert_big_endian(in);
  }

  void write_u8_register16(uint16_t a_register, uint8_t value) override {
    derived().enable();
    derived().write_byte(0x00);
    derived().write_byte16(a_register);
    derived().write_byte(value);
    derived().disable();
  }

  void write_s16_register16(uint16_t a_register, int16_t value) override {
    this->write_u16_register16(a_register, static_cast<uint16_t>(value));
  }

  void write_u16_register16(uint16_t a_register, uint16_t value) override {
    uint16_t out = convert_big_endian(value);
    derived().enable();
    derived().write_byte(0x00);
    derived().write_byte16(a_register);
    derived().write_array(reinterpret_cast<uint8_t *>(&out), sizeof(out));
    derived().disable();
  }

  void write_s32_register16(uint16_t a_register, int32_t value) override {
    this->write_u32_register16(a_register, static_cast<uint32_t>(value));
  }

  void write_u32_register16(uint16_t a_register, uint32_t value) override {
    uint32_t out = convert_big_endian(value);
    derived().enable();
    derived().write_byte(0x00);
    derived().write_byte16(a_register);
    derived().write_array(reinterpret_cast<uint8_t *>(&out), sizeof(out));
    derived().disable();
  }
};

}  // namespace ade78xx_base
}  // namespace esphome
