import esphome.codegen as cg
from esphome.components import ade78xx_base, ade7880_base, spi
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["spi"]
AUTO_LOAD = ["ade7880_base"]

ade7880_ns = cg.esphome_ns.namespace("ade7880_spi")
ADE7880SPI = ade7880_ns.class_(
    "ADE7880SPI", ade78xx_base.ADE78XXSPI, ade7880_base.ADE7880, spi.SPIDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ADE7880SPI),
        }
    )
    .extend(ade7880_base.ADE7880_CONFIG_SCHEMA)
    .extend(spi.spi_device_schema())
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await spi.register_spi_device(var, config)
    await ade7880_base.register_ade7880(var, config)
