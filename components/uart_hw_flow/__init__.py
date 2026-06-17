import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

uart_hw_flow_ns = cg.esphome_ns.namespace("uart_hw_flow")
UARTHwFlowComponent = uart_hw_flow_ns.class_("UARTHwFlowComponent", cg.Component)

CONF_UART_ID = "uart_id"
CONF_UART_NUM = "uart_num"
CONF_TX_PIN = "tx_pin"
CONF_RX_PIN = "rx_pin"
CONF_RTS_PIN = "rts_pin"
CONF_CTS_PIN = "cts_pin"
CONF_RX_FLOW_CTRL_THRESH = "rx_flow_ctrl_thresh"
CONF_EARLY_RTS_ASSERT = "early_rts_assert"


def validate_uart_num(value):
    value = cv.int_(value)
    if value < 0 or value > 2:
        raise cv.Invalid("uart_num must be 0, 1, or 2")
    return value


def validate_gpio_num(value):
    if isinstance(value, str):
        value = value.strip()
        if value.upper().startswith("GPIO"):
            value = value[4:]
    value = cv.int_(value)
    if value < 0 or value > 48:
        raise cv.Invalid("GPIO number must be between 0 and 48")
    return value


CONFIG_SCHEMA = cv.All(
    cv.only_on_esp32,
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(UARTHwFlowComponent),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Required(CONF_UART_NUM): validate_uart_num,
            cv.Required(CONF_TX_PIN): validate_gpio_num,
            cv.Required(CONF_RX_PIN): validate_gpio_num,
            cv.Required(CONF_RTS_PIN): validate_gpio_num,
            cv.Required(CONF_CTS_PIN): validate_gpio_num,
            cv.Optional(CONF_RX_FLOW_CTRL_THRESH, default=122): cv.int_range(min=1, max=127),
            cv.Optional(CONF_EARLY_RTS_ASSERT, default=True): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
)


async def to_code(config):
    cg.add_define("USE_UART_HW_FLOW")
    cg.add_global(cg.RawStatement('#include "esphome/components/uart_hw_flow/uart_hw_flow.h"'), prepend=True)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    uart_comp = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart(uart_comp))
    cg.add(var.set_uart_num(config[CONF_UART_NUM]))
    cg.add(var.set_pins(config[CONF_TX_PIN], config[CONF_RX_PIN], config[CONF_RTS_PIN], config[CONF_CTS_PIN]))
    cg.add(var.set_rx_flow_ctrl_thresh(config[CONF_RX_FLOW_CTRL_THRESH]))
    cg.add(var.set_early_rts_assert(config[CONF_EARLY_RTS_ASSERT]))
