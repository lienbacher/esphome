import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import spi
from esphome.const import (
    CONF_ID,
    CONF_GAIN,
)

DEPENDENCIES = ["spi"]
AUTO_LOAD = ["sensor", "voltage_sampler"]
MULTI_CONF = True
CODEOWNERS = ["@lienbacher"]

ads1256_ns = cg.esphome_ns.namespace("ads1256")
ADS1256Component = ads1256_ns.class_("ADS1256Component", cg.Component, spi.SPIDevice)

ADS1256Gain = ads1256_ns.enum("ADS1256Gain")
GAIN = {
    1: ADS1256Gain.ADS1256_GAIN_1,
    2: ADS1256Gain.ADS1256_GAIN_2,
    4: ADS1256Gain.ADS1256_GAIN_4,
    8: ADS1256Gain.ADS1256_GAIN_8,
    16: ADS1256Gain.ADS1256_GAIN_16,
    32: ADS1256Gain.ADS1256_GAIN_32,
    64: ADS1256Gain.ADS1256_GAIN_64,
}

def validate_gain(value):
    if (value%2) != 0 and value != 1:
      raise cv.Invalid(f'invalid gain "{value}", must be between 1 and 64 in 2^x steps')
    
    return cv.enum(GAIN)(value)

CONF_CONTINUOUS_MODE = "continuous_mode"
CONF_DRDY_PIN = "drdy_pin"
CONF_PDWN_PIN = "pdwn_pin"
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ADS1256Component),
            cv.Optional(CONF_CONTINUOUS_MODE, default=False): cv.boolean,
            cv.Required(CONF_DRDY_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_PDWN_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_GAIN): validate_gain,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    cg.add(var.set_continuous_mode(config[CONF_CONTINUOUS_MODE]))
    
    drdy = await cg.gpio_pin_expression(config[CONF_DRDY_PIN])
    cg.add(var.set_drdy_pin(drdy))
    cg.add(var.set_gain(config[CONF_GAIN]))
    #pdwn = await cg.gpio_pin_expression(config[CONF_PDWN_PIN])
    #cg.add(var.set_drdy_pin(pdwn))
