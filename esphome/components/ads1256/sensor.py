import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, voltage_sampler
from esphome.const import (
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
    CONF_ID,
)
from . import ads1256_ns, ADS1256Component

DEPENDENCIES = ["ads1256"]
CONF_MULTIPLEXER_POSITIVE = "multiplexer_positive"
CONF_MULTIPLEXER_NEGATIVE = "multiplexer_negative"
CONF_DATARATE = "datarate"

ADS1256MultiplexerPositive = ads1256_ns.enum("ADS1256MultiplexerPositive")
MUX_P = {
    "AIN0": ADS1256MultiplexerPositive.ADS1256_MUXP_AIN0,
    "AIN1": ADS1256MultiplexerPositive.ADS1256_MUXP_AIN1,
    "AIN2": ADS1256MultiplexerPositive.ADS1256_MUXP_AIN2,
    "AIN3": ADS1256MultiplexerPositive.ADS1256_MUXP_AIN3,
    "AIN4": ADS1256MultiplexerPositive.ADS1256_MUXP_AIN4,
    "AIN5": ADS1256MultiplexerPositive.ADS1256_MUXP_AIN5,
    "AIN6": ADS1256MultiplexerPositive.ADS1256_MUXP_AIN6,
    "AIN7": ADS1256MultiplexerPositive.ADS1256_MUXP_AIN7,
    "AINCOM": ADS1256MultiplexerPositive.ADS1256_MUXP_AINCOM,
}

ADS1256MultiplexerNegative = ads1256_ns.enum("ADS1256MultiplexerNegative")
MUX_N = {
    "AIN0": ADS1256MultiplexerNegative.ADS1256_MUXN_AIN0,
    "AIN1": ADS1256MultiplexerNegative.ADS1256_MUXN_AIN1,
    "AIN2": ADS1256MultiplexerNegative.ADS1256_MUXN_AIN2,
    "AIN3": ADS1256MultiplexerNegative.ADS1256_MUXN_AIN3,
    "AIN4": ADS1256MultiplexerNegative.ADS1256_MUXN_AIN4,
    "AIN5": ADS1256MultiplexerNegative.ADS1256_MUXN_AIN5,
    "AIN6": ADS1256MultiplexerNegative.ADS1256_MUXN_AIN6,
    "AIN7": ADS1256MultiplexerNegative.ADS1256_MUXN_AIN7,
    "AINCOM": ADS1256MultiplexerNegative.ADS1256_MUXN_AINCOM,
}

ADS1256Datarate = ads1256_ns.enum("ADS1256Datarate")
DATARATE = {
    30000: ADS1256Datarate.ADS1256_DATARATE_30000SPS,
    15000: ADS1256Datarate.ADS1256_DATARATE_15000SPS,
    7500: ADS1256Datarate.ADS1256_DATARATE_7500SPS,
    3750: ADS1256Datarate.ADS1256_DATARATE_3750SPS,
    2000: ADS1256Datarate.ADS1256_DATARATE_2000SPS,
    1000: ADS1256Datarate.ADS1256_DATARATE_1000SPS,
    500: ADS1256Datarate.ADS1256_DATARATE_500SPS,
    100: ADS1256Datarate.ADS1256_DATARATE_100SPS,
    60: ADS1256Datarate.ADS1256_DATARATE_60SPS,
    50: ADS1256Datarate.ADS1256_DATARATE_50SPS,
    30: ADS1256Datarate.ADS1256_DATARATE_30SPS3,
    25: ADS1256Datarate.ADS1256_DATARATE_25SPS,
    15: ADS1256Datarate.ADS1256_DATARATE_15SPS,
    10: ADS1256Datarate.ADS1256_DATARATE_10SPS,
    5: ADS1256Datarate.ADS1256_DATARATE_5SPS,
    2: ADS1256Datarate.ADS1256_DATARATE_2SPS,
}

ADS1256Sensor = ads1256_ns.class_(
    "ADS1256Sensor", sensor.Sensor, cg.PollingComponent, voltage_sampler.VoltageSampler
)

CONF_ADS1256_ID = "ads1256_id"
CONFIG_SCHEMA = (
    sensor.sensor_schema(
        ADS1256Sensor,
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.GenerateID(CONF_ADS1256_ID): cv.use_id(ADS1256Component),
            cv.Required(CONF_MULTIPLEXER_POSITIVE): cv.enum(MUX_P, upper=True, space="_"),
            cv.Required(CONF_MULTIPLEXER_NEGATIVE): cv.enum(MUX_N, upper=True, space="_"),
            cv.Optional(CONF_DATARATE): cv.enum(DATARATE),
        }
    )
    .extend(cv.polling_component_schema("60s"))
)


async def to_code(config):
    paren = await cg.get_variable(config[CONF_ADS1256_ID])
    var = cg.new_Pvariable(config[CONF_ID], paren)
    await sensor.register_sensor(var, config)
    await cg.register_component(var, config)

    cg.add(var.set_multiplexer_p(config[CONF_MULTIPLEXER_POSITIVE]))
    cg.add(var.set_multiplexer_n(config[CONF_MULTIPLEXER_NEGATIVE]))
    
    cg.add(var.set_datarate(config[CONF_DATARATE]))

    cg.add(paren.register_sensor(var))
