#pragma once

// Source: https://www.analog.com/media/en/technical-documentation/data-sheets/ADE7816.pdf

namespace esphome {
namespace ade7816_base {

constexpr uint16_t VGAIN = 0x4380;
constexpr uint16_t IAGAIN = 0x4381;
constexpr uint16_t IBGAIN = 0x4382;
constexpr uint16_t ICGAIN = 0x4383;
constexpr uint16_t IDGAIN = 0x4384;
constexpr uint16_t IEGAIN = 0x4385;
constexpr uint16_t IFGAIN = 0x4386;

constexpr uint16_t DICOEFF = 0x4388;

constexpr uint16_t HPFDIS = 0x4389;
constexpr uint16_t VRMSOS = 0x438A;
constexpr uint16_t IARMSOS = 0x438B;
constexpr uint16_t IBRMSOS = 0x438C;
constexpr uint16_t ICRMSOS = 0x438D;
constexpr uint16_t IDRMSOS = 0x438E;
constexpr uint16_t IERMSOS = 0x438F;
constexpr uint16_t IFRMSOS = 0x4390;

constexpr uint16_t AWGAIN = 0x4391;
constexpr uint16_t AWATTOS = 0x4392;
constexpr uint16_t BWGAIN = 0x4393;
constexpr uint16_t BWATTOS = 0x4394;
constexpr uint16_t CWGAIN = 0x4395;
constexpr uint16_t CWATTOS = 0x4396;
constexpr uint16_t DWGAIN = 0x4397;
constexpr uint16_t DWATTOS = 0x4398;
constexpr uint16_t EWGAIN = 0x4399;
constexpr uint16_t EWATTOS = 0x439A;
constexpr uint16_t FWGAIN = 0x439B;
constexpr uint16_t FWATTOS = 0x439C;

constexpr uint16_t AVARGAIN = 0x439D;
constexpr uint16_t AVAROS = 0x439E;
constexpr uint16_t BVARGAIN = 0x439F;
constexpr uint16_t BVAROS = 0x43A0;
constexpr uint16_t CVARGAIN = 0x43A1;
constexpr uint16_t CVAROS = 0x43A2;
constexpr uint16_t DVARGAIN = 0x43A3;
constexpr uint16_t DVAROS = 0x43A4;
constexpr uint16_t EVARGAIN = 0x43A5;
constexpr uint16_t EVAROS = 0x43A6;
constexpr uint16_t FVARGAIN = 0x43A7;
constexpr uint16_t FVAROS = 0x43A8;

constexpr uint16_t WTHR1 = 0x43AB;
constexpr uint16_t WTHR0 = 0x43AC;

constexpr uint16_t VARTHR1 = 0x43AD;
constexpr uint16_t VARTHR0 = 0x43AE;

constexpr uint16_t APNOLOAD = 0x43AF;
constexpr uint16_t VARNOLOAD = 0x43B0;

constexpr uint16_t PCF_A_COEFF = 0x43B1;
constexpr uint16_t PCF_B_COEFF = 0x43B2;
constexpr uint16_t PCF_C_COEFF = 0x43B3;
constexpr uint16_t PCF_D_COEFF = 0x43B4;
constexpr uint16_t PCF_E_COEFF = 0x43B5;
constexpr uint16_t PCF_F_COEFF = 0x43B6;

constexpr uint16_t VRMS = 0x43C0;
constexpr uint16_t IARMS = 0x43C1;
constexpr uint16_t IBRMS = 0x43C2;
constexpr uint16_t ICRMS = 0x43C3;
constexpr uint16_t IDRMS = 0x43C4;
constexpr uint16_t IERMS = 0x43C5;
constexpr uint16_t IFRMS = 0x43C6;

constexpr uint16_t RUN = 0xE228;

constexpr uint16_t AWATTHR = 0xE400;
constexpr uint16_t BWATTHR = 0xE401;
constexpr uint16_t CWATTHR = 0xE402;
constexpr uint16_t DWATTHR = 0xE403;
constexpr uint16_t EWATTHR = 0xE404;
constexpr uint16_t FWATTHR = 0xE405;

constexpr uint16_t AVARHR = 0xE406;
constexpr uint16_t BVARHR = 0xE407;
constexpr uint16_t CVARHR = 0xE408;
constexpr uint16_t DVARHR = 0xE409;
constexpr uint16_t EVARHR = 0xE40A;
constexpr uint16_t FVARHR = 0xE40B;

constexpr uint16_t IPEAK = 0xE500;
constexpr uint16_t VPEAK = 0xE501;
constexpr uint16_t STATUS0 = 0xE502;
constexpr uint16_t STATUS1 = 0xE503;

constexpr uint16_t OILVL = 0xE507;
constexpr uint16_t OVLVL = 0xE508;
constexpr uint16_t SAGLVL = 0xE509;

constexpr uint16_t MASK0 = 0xE50A;
constexpr uint16_t MASK1 = 0xE50B;

constexpr uint16_t IAWV_IDWV = 0xE50C;
constexpr uint16_t IBWV_IEWV = 0xE50D;
constexpr uint16_t ICWV_IFWV = 0xE50E;

constexpr uint16_t VWV = 0xE510;

constexpr uint16_t CHECKSUM = 0xE51F;

constexpr uint16_t CHSTATUS = 0xE600;
constexpr uint16_t ANGLE0 = 0xE601;
constexpr uint16_t ANGLE1 = 0xE602;
constexpr uint16_t ANGLE2 = 0xE603;

constexpr uint16_t PERIOD = 0xE607;
constexpr uint16_t CHNOLOAD = 0xE608;

constexpr uint16_t LINECYC = 0xE60C;
constexpr uint16_t ZXTOUT = 0xE60D;
constexpr uint16_t COMPMODE = 0xE60E;
constexpr uint16_t GAIN = 0xE60F;

constexpr uint16_t CHSIGN = 0xE617;
constexpr uint16_t CONFIG = 0xE618;

constexpr uint16_t MMODE = 0xE700;
constexpr uint16_t ACCMODE = 0xE701;
constexpr uint16_t LCYCMODE = 0xE702;
constexpr uint16_t PEAKCYC = 0xE703;
constexpr uint16_t SAGCYC = 0xE704;

constexpr uint16_t HSDC_CFG = 0xE706;
constexpr uint16_t VERSION = 0xE707;

constexpr uint16_t DSPWP_SET = 0xE7E3;
constexpr uint16_t DSPWP_SEL = 0xE7FE;

// This address can be used in manipulating the SS/HSA pin when SPI is chosen as the active port.
constexpr uint16_t RESERVED_EBFF = 0xEBFF;

constexpr uint16_t CONFIG2 = 0xEC01;

// STATUS1 Register Bits
constexpr uint32_t STATUS1_RSTDONE = (1 << 15);

// CONFIG Register Bits
constexpr uint16_t CONFIG_SWRST = (1 << 7);

// CONFIG2 Register Bits
constexpr uint8_t CONFIG2_I2C_LOCK = (1 << 1);
constexpr uint8_t CONFIG2_SPI_LOCK = (0 << 1);

// COMPMODE Register Bits
constexpr uint16_t COMPMODE_DEFAULT = 0x01FF;
constexpr uint16_t COMPMODE_CHANNEL_SEL = (1 << 14);

// RUN Register Bits
constexpr uint16_t RUN_ENABLE = (1 << 0);

// DSPWP_SET Register Bits
constexpr uint8_t DSPWP_SET_RO = (1 << 7);

// DSPWP_SEL Register Bits
constexpr uint8_t DSPWP_SEL_SET = 0xAD;

}  // namespace ade7816_base
}  // namespace esphome
