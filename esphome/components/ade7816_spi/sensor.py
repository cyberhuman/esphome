import esphome.codegen as cg
from esphome.components import ade78xx_base, ade7816_base, spi
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["spi"]
AUTO_LOAD = ["ade7816_base"]

ade7816_ns = cg.esphome_ns.namespace("ade7816_spi")
ADE7816 = ade7816_ns.class_(
    "ADE7816SPI", ade78xx_base.ADE78XXSPI, ade7816_base.ADE7816, spi.SPIDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ADE7816),
        }
    )
    .extend(ade7816_base.ADE7816_CONFIG_SCHEMA)
    .extend(spi.spi_device_schema())
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await spi.register_spi_device(var, config)
    await ade7816_base.register_ade7816(var, config)
