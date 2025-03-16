import esphome.codegen as cg
from esphome.components import ade78xx_base
import esphome.config_validation as cv
from esphome.const import CONF_PHASE_A, CONF_PHASE_B, CONF_PHASE_C

CODEOWNERS = ["@kpfleming", "@cyberhuman"]

AUTO_LOAD = ["ade78xx_base"]

ade7880_base_ns = cg.esphome_ns.namespace("ade7880_base")
ADE7880 = ade7880_base_ns.class_("ADE7880", ade78xx_base.ADE78XX)

POWER_CHANNEL_SCHEMA = ade78xx_base.channel_schema(
    include_voltage=True,
    include_current=True,
    include_active_power=True,
    include_apparent_power=True,
    include_power_factor=True,
    include_forward_active_energy=True,
    include_reverse_active_energy=True,
    include_current_gain_calibration=True,
    include_voltage_gain_calibration=True,
    include_power_gain_calibration=True,
    include_phase_angle_calibration=True,
)

NEUTRAL_CHANNEL_SCHEMA = ade78xx_base.channel_schema(
    include_current=True,
    include_current_gain_calibration=True,
)

ADE7880_CONFIG_SCHEMA = ade78xx_base.ADE78XX_CONFIG_SCHEMA.extend(
    cv.Schema(
        {
            cv.Optional(CONF_PHASE_A): POWER_CHANNEL_SCHEMA,
            cv.Optional(CONF_PHASE_B): POWER_CHANNEL_SCHEMA,
            cv.Optional(CONF_PHASE_C): POWER_CHANNEL_SCHEMA,
            cv.Optional(ade78xx_base.CONF_NEUTRAL): NEUTRAL_CHANNEL_SCHEMA,
        }
    )
).extend(cv.polling_component_schema("60s"))

channels = {
    CONF_PHASE_A: ade78xx_base.ChannelConfig(
        name="Phase A",
        igain=ade7880_base_ns.namespace("AIGAIN"),
        vgain=ade7880_base_ns.namespace("AVGAIN"),
        pgain=ade7880_base_ns.namespace("APGAIN"),
        phcal=ade7880_base_ns.namespace("APHCAL"),
        irms=ade7880_base_ns.namespace("AIRMS"),
        vrms=ade7880_base_ns.namespace("AVRMS"),
        watt=ade7880_base_ns.namespace("AWATT"),
        va=ade7880_base_ns.namespace("AVA"),
        pf=ade7880_base_ns.namespace("APF"),
        fwatthr=ade7880_base_ns.namespace("AFWATTHR"),
        fvarhr=ade7880_base_ns.namespace("AFVARHR"),
    ),
    CONF_PHASE_B: ade78xx_base.ChannelConfig(
        name="Phase B",
        igain=ade7880_base_ns.namespace("BIGAIN"),
        vgain=ade7880_base_ns.namespace("BVGAIN"),
        pgain=ade7880_base_ns.namespace("BPGAIN"),
        phcal=ade7880_base_ns.namespace("BPHCAL"),
        irms=ade7880_base_ns.namespace("BIRMS"),
        vrms=ade7880_base_ns.namespace("BVRMS"),
        watt=ade7880_base_ns.namespace("BWATT"),
        va=ade7880_base_ns.namespace("BVA"),
        pf=ade7880_base_ns.namespace("BPF"),
        fwatthr=ade7880_base_ns.namespace("BFWATTHR"),
        fvarhr=ade7880_base_ns.namespace("BFVARHR"),
    ),
    CONF_PHASE_C: ade78xx_base.ChannelConfig(
        name="Phase C",
        igain=ade7880_base_ns.namespace("CIGAIN"),
        vgain=ade7880_base_ns.namespace("CVGAIN"),
        pgain=ade7880_base_ns.namespace("CPGAIN"),
        phcal=ade7880_base_ns.namespace("CPHCAL"),
        irms=ade7880_base_ns.namespace("CIRMS"),
        vrms=ade7880_base_ns.namespace("CVRMS"),
        watt=ade7880_base_ns.namespace("CWATT"),
        va=ade7880_base_ns.namespace("CVA"),
        pf=ade7880_base_ns.namespace("CPF"),
        fwatthr=ade7880_base_ns.namespace("CFWATTHR"),
        fvarhr=ade7880_base_ns.namespace("CFVARHR"),
    ),
    ade78xx_base.CONF_NEUTRAL: ade78xx_base.ChannelConfig(
        name="Neutral",
        igain=ade7880_base_ns.namespace("NIGAIN"),
        irms=ade7880_base_ns.namespace("NIRMS"),
    ),
}


async def register_ade7880(var, config):
    await ade78xx_base.register_ade78xx(var, config, channels)
