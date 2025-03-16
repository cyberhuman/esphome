import esphome.codegen as cg
from esphome.components import ade78xx_base
import esphome.config_validation as cv
from esphome.const import CONF_VOLTAGE

CODEOWNERS = ["@cyberhuman"]

AUTO_LOAD = ["ade78xx_base"]

CONF_CURRENT_A = "current_a"
CONF_CURRENT_B = "current_b"
CONF_CURRENT_C = "current_c"
CONF_CURRENT_D = "current_d"
CONF_CURRENT_E = "current_e"
CONF_CURRENT_F = "current_f"

ade7816_base_ns = cg.esphome_ns.namespace("ade7816_base")
ADE7816 = ade7816_base_ns.class_("ADE7816", ade78xx_base.ADE78XX)

CURRENT_CHANNEL_SCHEMA = ade78xx_base.channel_schema(
    include_current=True,
    include_forward_active_energy=True,
    include_reverse_active_energy=True,
    include_current_gain_calibration=True,
    include_power_gain_calibration=True,
    include_phase_angle_calibration=True,
)

VOLTAGE_CHANNEL_SCHEMA = ade78xx_base.channel_schema(
    include_voltage=True,
    include_voltage_gain_calibration=True,
)

ADE7816_CONFIG_SCHEMA = ade78xx_base.ADE78XX_CONFIG_SCHEMA.extend(
    cv.Schema(
        {
            cv.Optional(CONF_CURRENT_A): CURRENT_CHANNEL_SCHEMA,
            cv.Optional(CONF_CURRENT_B): CURRENT_CHANNEL_SCHEMA,
            cv.Optional(CONF_CURRENT_C): CURRENT_CHANNEL_SCHEMA,
            cv.Optional(CONF_CURRENT_D): CURRENT_CHANNEL_SCHEMA,
            cv.Optional(CONF_CURRENT_E): CURRENT_CHANNEL_SCHEMA,
            cv.Optional(CONF_CURRENT_F): CURRENT_CHANNEL_SCHEMA,
            cv.Optional(CONF_VOLTAGE): VOLTAGE_CHANNEL_SCHEMA,
        }
    )
).extend(cv.polling_component_schema("60s"))

channels = {
    CONF_CURRENT_A: ade78xx_base.ChannelConfig(
        name="Current A",
        igain=ade7816_base_ns.namespace("IAGAIN"),
        pgain=ade7816_base_ns.namespace("AWGAIN"),
        phcal_24bit=ade7816_base_ns.namespace("PCF_A_COEFF"),
        irms=ade7816_base_ns.namespace("IARMS"),
        fwatthr=ade7816_base_ns.namespace("AWATTHR"),
        fvarhr=ade7816_base_ns.namespace("AVARHR"),
    ),
    CONF_CURRENT_B: ade78xx_base.ChannelConfig(
        name="Current B",
        igain=ade7816_base_ns.namespace("IBGAIN"),
        pgain=ade7816_base_ns.namespace("BWGAIN"),
        phcal_24bit=ade7816_base_ns.namespace("PCF_B_COEFF"),
        irms=ade7816_base_ns.namespace("IBRMS"),
        fwatthr=ade7816_base_ns.namespace("BWATTHR"),
        fvarhr=ade7816_base_ns.namespace("BVARHR"),
    ),
    CONF_CURRENT_C: ade78xx_base.ChannelConfig(
        name="Current C",
        igain=ade7816_base_ns.namespace("ICGAIN"),
        pgain=ade7816_base_ns.namespace("CWGAIN"),
        phcal_24bit=ade7816_base_ns.namespace("PCF_C_COEFF"),
        irms=ade7816_base_ns.namespace("ICRMS"),
        fwatthr=ade7816_base_ns.namespace("CWATTHR"),
        fvarhr=ade7816_base_ns.namespace("CVARHR"),
    ),
    CONF_CURRENT_D: ade78xx_base.ChannelConfig(
        name="Current D",
        igain=ade7816_base_ns.namespace("IDGAIN"),
        pgain=ade7816_base_ns.namespace("DWGAIN"),
        phcal_24bit=ade7816_base_ns.namespace("PCF_D_COEFF"),
        irms=ade7816_base_ns.namespace("IDRMS"),
        fwatthr=ade7816_base_ns.namespace("DWATTHR"),
        fvarhr=ade7816_base_ns.namespace("DVARHR"),
    ),
    CONF_CURRENT_E: ade78xx_base.ChannelConfig(
        name="Current E",
        igain=ade7816_base_ns.namespace("IEGAIN"),
        pgain=ade7816_base_ns.namespace("EWGAIN"),
        phcal_24bit=ade7816_base_ns.namespace("PCF_E_COEFF"),
        irms=ade7816_base_ns.namespace("IERMS"),
        fwatthr=ade7816_base_ns.namespace("EWATTHR"),
        fvarhr=ade7816_base_ns.namespace("EVARHR"),
    ),
    CONF_CURRENT_F: ade78xx_base.ChannelConfig(
        name="Current F",
        igain=ade7816_base_ns.namespace("IFGAIN"),
        pgain=ade7816_base_ns.namespace("FWGAIN"),
        phcal_24bit=ade7816_base_ns.namespace("PCF_F_COEFF"),
        irms=ade7816_base_ns.namespace("IFRMS"),
        fwatthr=ade7816_base_ns.namespace("FWATTHR"),
        fvarhr=ade7816_base_ns.namespace("FVARHR"),
    ),
    CONF_VOLTAGE: ade78xx_base.ChannelConfig(
        name="Voltage",
        vgain=ade7816_base_ns.namespace("VGAIN"),
        vrms=ade7816_base_ns.namespace("VRMS"),
    ),
}


async def register_ade7816(var, config):
    await ade78xx_base.register_ade78xx(var, config, channels)
