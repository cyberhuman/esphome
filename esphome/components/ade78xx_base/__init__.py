from dataclasses import dataclass
from typing import Optional

from esphome import pins
import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ACTIVE_POWER,
    CONF_APPARENT_POWER,
    CONF_CALIBRATION,
    CONF_CURRENT,
    CONF_FORWARD_ACTIVE_ENERGY,
    CONF_FREQUENCY,
    CONF_ID,
    CONF_NAME,
    CONF_PHASE_ANGLE,
    CONF_POWER_FACTOR,
    CONF_RESET_PIN,
    CONF_REVERSE_ACTIVE_ENERGY,
    CONF_VOLTAGE,
    CONF_VOLTAGE_GAIN,
    DEVICE_CLASS_APPARENT_POWER,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_POWER_FACTOR,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_VOLT_AMPS,
    UNIT_VOLT_AMPS_REACTIVE_HOURS,
    UNIT_WATT,
    UNIT_WATT_HOURS,
)

CODEOWNERS = ["@kpfleming", "@cyberhuman"]

ade78xx_base_ns = cg.esphome_ns.namespace("ade78xx_base")
ADE78XX = ade78xx_base_ns.class_("ADE78xx", cg.PollingComponent)
ADE78XXI2C = ade78xx_base_ns.class_("ADE78xxI2C")
ADE78XXSPI = ade78xx_base_ns.class_("ADE78xxSPI")
Channel = ade78xx_base_ns.struct("Channel")

CONF_CURRENT_GAIN = "current_gain"
CONF_IRQ0_PIN = "irq0_pin"
CONF_IRQ1_PIN = "irq1_pin"
CONF_POWER_GAIN = "power_gain"

CONF_NEUTRAL = "neutral"


@dataclass
class ChannelConfig:
    name: str
    igain: Optional[cg.MockObj] = None
    vgain: Optional[cg.MockObj] = None
    pgain: Optional[cg.MockObj] = None
    phcal_10bit: Optional[cg.MockObj] = None
    phcal_24bit: Optional[cg.MockObj] = None
    irms: Optional[cg.MockObj] = None
    vrms: Optional[cg.MockObj] = None
    watt: Optional[cg.MockObj] = None
    va: Optional[cg.MockObj] = None
    pf: Optional[cg.MockObj] = None
    fwatthr: Optional[cg.MockObj] = None
    fvarhr: Optional[cg.MockObj] = None


def channel_schema(
    include_voltage=False,
    include_current=False,
    include_active_power=False,
    include_apparent_power=False,
    include_power_factor=False,
    include_forward_active_energy=False,
    include_reverse_active_energy=False,
    include_current_gain_calibration=False,
    include_voltage_gain_calibration=False,
    include_power_gain_calibration=False,
    include_phase_angle_calibration=False,
):
    calibration_schema = {
        CONF_CURRENT_GAIN: cv.int_ if include_current_gain_calibration else None,
        CONF_VOLTAGE_GAIN: cv.int_ if include_voltage_gain_calibration else None,
        CONF_POWER_GAIN: cv.int_ if include_power_gain_calibration else None,
        CONF_PHASE_ANGLE: cv.int_ if include_phase_angle_calibration else None,
    }

    calibration_schema = {
        cv.Required(key): value
        for key, value in calibration_schema.items()
        if value is not None
    }

    sensor_schema = {
        CONF_VOLTAGE: sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        )
        if include_voltage
        else None,
        CONF_CURRENT: sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        )
        if include_current
        else None,
        CONF_ACTIVE_POWER: sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        )
        if include_active_power
        else None,
        CONF_APPARENT_POWER: sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT_AMPS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_APPARENT_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        )
        if include_apparent_power
        else None,
        CONF_POWER_FACTOR: sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER_FACTOR,
            state_class=STATE_CLASS_MEASUREMENT,
        )
        if include_power_factor
        else None,
        CONF_FORWARD_ACTIVE_ENERGY: sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT_HOURS,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        )
        if include_forward_active_energy
        else None,
        CONF_REVERSE_ACTIVE_ENERGY: sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT_AMPS_REACTIVE_HOURS,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        )
        if include_reverse_active_energy
        else None,
    }

    return cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Channel),
            cv.Optional(CONF_NAME): cv.string_strict,
            cv.Required(CONF_CALIBRATION): cv.Schema(calibration_schema),
        }
    ).extend(
        cv.Schema(
            {
                cv.Optional(key): cv.maybe_simple_value(value, key=CONF_NAME)
                for key, value in sensor_schema.items()
                if value is not None
            }
        )
    )


ADE78XX_CONFIG_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_FREQUENCY, default="50Hz"): cv.All(
            cv.frequency, cv.Range(min=45.0, max=66.0)
        ),
        cv.Optional(CONF_IRQ0_PIN): pins.internal_gpio_input_pin_schema,
        cv.Required(CONF_IRQ1_PIN): pins.internal_gpio_input_pin_schema,
        cv.Optional(CONF_RESET_PIN): pins.internal_gpio_output_pin_schema,
    }
).extend(cv.polling_component_schema("60s"))


async def make_channel(config, channel_config):
    var = cg.new_Pvariable(
        config[CONF_ID],
        channel_config.name,
    )
    if channel_config.igain:
        cg.add(var.set_igain(channel_config.igain))
    if channel_config.vgain:
        cg.add(var.set_vgain(channel_config.vgain))
    if channel_config.pgain:
        cg.add(var.set_pgain(channel_config.pgain))
    if channel_config.phcal_10bit:
        cg.add(var.set_phcal_10bit(channel_config.phcal_10bit))
    if channel_config.phcal_24bit:
        cg.add(var.set_phcal_24bit(channel_config.phcal_24bit))
    if channel_config.irms:
        cg.add(var.set_irms(channel_config.irms))
    if channel_config.vrms:
        cg.add(var.set_vrms(channel_config.vrms))
    if channel_config.watt:
        cg.add(var.set_watt(channel_config.watt))
    if channel_config.va:
        cg.add(var.set_va(channel_config.va))
    if channel_config.pf:
        cg.add(var.set_pf(channel_config.pf))
    if channel_config.fwatthr:
        cg.add(var.set_fwatthr(channel_config.fwatthr))
    if channel_config.fvarhr:
        cg.add(var.set_fvarhr(channel_config.fvarhr))

    channel_name = config.get(CONF_NAME)
    for sensor_type in [
        CONF_CURRENT,
        CONF_VOLTAGE,
        CONF_ACTIVE_POWER,
        CONF_APPARENT_POWER,
        CONF_POWER_FACTOR,
        CONF_FORWARD_ACTIVE_ENERGY,
        CONF_REVERSE_ACTIVE_ENERGY,
    ]:
        if conf := config.get(sensor_type):
            sensor_name = conf.get(CONF_NAME)
            if (
                sensor_name
                and channel_name
                and not sensor_name.startswith(channel_name)
            ):
                conf[CONF_NAME] = f"{channel_name} {sensor_name}"

            sens = await sensor.new_sensor(conf)
            cg.add(getattr(var, f"set_{sensor_type}")(sens))

    for calib_type in [
        CONF_CURRENT_GAIN,
        CONF_VOLTAGE_GAIN,
        CONF_POWER_GAIN,
        CONF_PHASE_ANGLE,
    ]:
        if conf := config[CONF_CALIBRATION].get(calib_type):
            cg.add(getattr(var, f"set_{calib_type}_calibration")(conf))

    return var


async def register_ade78xx(var, config, channels):
    await cg.register_component(var, config)

    if irq0_pin := config.get(CONF_IRQ0_PIN):
        pin = await cg.gpio_pin_expression(irq0_pin)
        cg.add(var.set_irq0_pin(pin))

    pin = await cg.gpio_pin_expression(config[CONF_IRQ1_PIN])
    cg.add(var.set_irq1_pin(pin))

    if reset_pin := config.get(CONF_RESET_PIN):
        pin = await cg.gpio_pin_expression(reset_pin)
        cg.add(var.set_reset_pin(pin))

    if frequency := config.get(CONF_FREQUENCY):
        cg.add(var.set_frequency(frequency))

    channels_ = []
    for channel, registers in channels.items():
        if channel := config.get(channel):
            chan = await make_channel(channel, registers)
            channels_.append(chan)
    cg.add(var.set_channels(channels_))
