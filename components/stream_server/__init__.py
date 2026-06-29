from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import uart
from esphome.const import CONF_ID, CONF_PORT, CONF_BUFFER_SIZE

# ESPHome doesn't know the Stream abstraction yet, so hardcode to use a UART for now.
stream_server_ns = cg.esphome_ns.namespace("stream_server")
StreamServerComponent = stream_server_ns.class_("StreamServerComponent", cg.Component)
PauseAction = stream_server_ns.class_("PauseAction", automation.Action)
ResumeAction = stream_server_ns.class_("ResumeAction", automation.Action)

AUTO_LOAD = ["socket"]

DEPENDENCIES = ["uart", "network"]

MULTI_CONF = True

def validate_buffer_size(buffer_size):
    if buffer_size & (buffer_size - 1) != 0:
        raise cv.Invalid("Buffer size must be a power of two.")
    return buffer_size


CONFIG_SCHEMA = cv.All(
    cv.require_esphome_version(2022, 3, 0),
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(StreamServerComponent),
            cv.Optional(CONF_PORT, default=6638): cv.port,
            cv.Optional(CONF_BUFFER_SIZE, default=128): cv.All(
                cv.positive_int, validate_buffer_size
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_buffer_size(config[CONF_BUFFER_SIZE]))

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

@automation.register_action(
    "stream_server.pause",
    PauseAction,
    cv.Schema({cv.Required(CONF_ID): cv.use_id(StreamServerComponent)}),
    synchronous=True,
)
async def stream_server_pause_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)


@automation.register_action(
    "stream_server.resume",
    ResumeAction,
    cv.Schema({cv.Required(CONF_ID): cv.use_id(StreamServerComponent)}),
    synchronous=True,
)
async def stream_server_resume_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)

