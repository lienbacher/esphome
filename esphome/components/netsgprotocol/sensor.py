import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_TEMPERATURE,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_WATT,
    UNIT_WATT_HOURS,
    UNIT_CELSIUS,
)
from . import CONF_NetSGProtocol_ID, NetSGProtocolComponent

DEPENDENCIES = ["netsgprotocol"]
CONF_DC_VOLTAGE = "dc_voltage"
CONF_DC_CURRENT = "dc_current"
CONF_DC_POWER = "dc_power"
CONF_AC_VOLTAGE = "ac_voltage"
CONF_AC_CURRENT = "ac_current"
CONF_AC_POWER = "ac_power"
CONF_POWER_GEN_TOTAL = "power_gen_total"
CONF_DEVICE_TEMPERATURE = "device_temperature"

CONFIG_SCHEMA = {
    cv.GenerateID(CONF_NetSGProtocol_ID): cv.use_id(NetSGProtocolComponent),
    cv.Optional(CONF_DC_VOLTAGE): sensor.sensor_schema(
        device_class=DEVICE_CLASS_VOLTAGE, unit_of_measurement=UNIT_VOLT
    ),
    cv.Optional(CONF_DC_CURRENT): sensor.sensor_schema(
        device_class=DEVICE_CLASS_CURRENT, unit_of_measurement=UNIT_AMPERE
    ),
    cv.Optional(CONF_DC_POWER): sensor.sensor_schema(
        device_class=DEVICE_CLASS_POWER, unit_of_measurement=UNIT_WATT
    ),
    cv.Optional(CONF_AC_VOLTAGE): sensor.sensor_schema(
        device_class=DEVICE_CLASS_VOLTAGE, unit_of_measurement=UNIT_VOLT
    ),
    cv.Optional(CONF_AC_CURRENT): sensor.sensor_schema(
        device_class=DEVICE_CLASS_CURRENT, unit_of_measurement=UNIT_AMPERE
    ),
    cv.Optional(CONF_AC_POWER): sensor.sensor_schema(
        device_class=DEVICE_CLASS_POWER, unit_of_measurement=UNIT_WATT
    ),
    cv.Optional(CONF_POWER_GEN_TOTAL): sensor.sensor_schema(
        device_class=DEVICE_CLASS_ENERGY, unit_of_measurement=UNIT_WATT_HOURS
    ),
    cv.Optional(CONF_DEVICE_TEMPERATURE): sensor.sensor_schema(
        device_class=DEVICE_CLASS_TEMPERATURE, unit_of_measurement=UNIT_CELSIUS
    ),
}


async def to_code(config):
    netsgprotocol_component = await cg.get_variable(config[CONF_NetSGProtocol_ID])
    if CONF_DC_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_DC_VOLTAGE])
        cg.add(netsgprotocol_component.set_dc_voltage_sensor(sens))
    if CONF_DC_CURRENT in config:
        sens = await sensor.new_sensor(config[CONF_DC_CURRENT])
        cg.add(netsgprotocol_component.set_dc_current_sensor(sens))
    if CONF_DC_POWER in config:
        sens = await sensor.new_sensor(config[CONF_DC_POWER])
        cg.add(netsgprotocol_component.set_dc_power_sensor(sens))
    if CONF_AC_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_AC_VOLTAGE])
        cg.add(netsgprotocol_component.set_ac_voltage_sensor(sens))
    if CONF_AC_CURRENT in config:
        sens = await sensor.new_sensor(config[CONF_AC_CURRENT])
        cg.add(netsgprotocol_component.set_ac_current_sensor(sens))
    if CONF_AC_POWER in config:
        sens = await sensor.new_sensor(config[CONF_AC_POWER])
        cg.add(netsgprotocol_component.set_ac_power_sensor(sens))
    if CONF_POWER_GEN_TOTAL in config:
        sens = await sensor.new_sensor(config[CONF_POWER_GEN_TOTAL])
        cg.add(netsgprotocol_component.set_power_gen_total_sensor(sens))
    if CONF_DEVICE_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_DEVICE_TEMPERATURE])
        cg.add(netsgprotocol_component.set_device_temperature_sensor(sens))
