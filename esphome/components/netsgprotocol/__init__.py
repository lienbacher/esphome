import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.components import switch
from esphome.const import CONF_ID
from esphome import pins, automation
from esphome.automation import maybe_simple_id

DEPENDENCIES = ["uart"]
CODEOWNERS = ["@lienbacher"]

AUTO_LOAD = ["sensor"]

netsgprotocol_ns = cg.esphome_ns.namespace("netsgprotocol")
NetSGProtocolComponent = netsgprotocol_ns.class_(
    "NetSGProtocolComponent", cg.PollingComponent, uart.UARTDevice, switch.Switch
)
NetSGProtocolRestart = netsgprotocol_ns.class_(
    "NetSGProtocolRestart", automation.Action
)
CONF_NetSGProtocol_ID = "netsgprotocol_id"
CONF_INVERTER_DEVICE_ID = "inverter_device_id"
CONF_POLL_INTERVAL = "poll_interval"
CONF_POWER_GRADE = "power_grade"
CONF_SET_PIN = "set_pin"
CONF_ON_OFF_SWITCH = "on_off_switch"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(NetSGProtocolComponent),
            cv.Required(CONF_INVERTER_DEVICE_ID): cv.int_range(min=1, max=4294967295),
            cv.Optional(CONF_POLL_INTERVAL, default=1): cv.int_range(min=1, max=3600),
            cv.Optional(CONF_POWER_GRADE, default=100): cv.int_range(min=1, max=100),
            cv.Optional(CONF_SET_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_ON_OFF_SWITCH): cv.ensure_list(
                switch.switch_schema(switch.Switch)
            ),
        }
    ).extend(uart.UART_DEVICE_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "netsgprotocol",
    baud_rate=9600,
    require_tx=True,
    require_rx=True,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    await switch.new_switch(config)
    cg.add(var.set_inverter_device_id(config[CONF_INVERTER_DEVICE_ID]))
    cg.add(var.set_poll_interval(config[CONF_POLL_INTERVAL]))
    cg.add(var.set_power_grade(config[CONF_POWER_GRADE]))
    set_pin = await cg.gpio_pin_expression(config[CONF_SET_PIN])
    cg.add(var.set_set_pin(set_pin))


CALIBRATION_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(NetSGProtocolComponent),
    }
)
