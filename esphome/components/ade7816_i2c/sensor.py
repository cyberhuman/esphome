import esphome.codegen as cg
from esphome.components import ade78xx_base, ade7816_base, i2c
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["ade7816_base"]

ade7816_ns = cg.esphome_ns.namespace("ade7816_i2c")
ADE7816 = ade7816_ns.class_(
    "ADE7816I2C", ade78xx_base.ADE78XXI2C, ade7816_base.ADE7816, i2c.I2CDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ADE7816),
        }
    )
    .extend(ade7816_base.ADE7816_CONFIG_SCHEMA)
    .extend(i2c.i2c_device_schema(0x38))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await i2c.register_i2c_device(var, config)
    await ade7816_base.register_ade7816(var, config)
