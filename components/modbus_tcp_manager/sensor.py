import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_UPDATE_INTERVAL,
)

# Configuration constants
CONF_MODBUS_TCP_ID = "modbus_tcp_id"
CONF_REGISTER_ADDRESS = "register_address"
CONF_FUNCTION_CODE = "function_code"
CONF_SCALE = "scale"
CONF_OFFSET = "offset"
CONF_UNIT_ID = "unit_id"
CONF_REGISTERS_COUNT = "registers_count"

# Import from main component
from . import modbus_tcp_ns, ModbusTCPManager

ModbusTCPSensor = modbus_tcp_ns.class_("ModbusTCPSensor", cg.PollingComponent, sensor.Sensor)

# Dependencies
DEPENDENCIES = ["network"]

# Configuration schema
CONFIG_SCHEMA = sensor.sensor_schema(ModbusTCPSensor).extend({
    cv.GenerateID(CONF_MODBUS_TCP_ID): cv.use_id(ModbusTCPManager),
    cv.Required(CONF_REGISTER_ADDRESS): cv.positive_int,
    cv.Optional(CONF_FUNCTION_CODE, default=3): cv.one_of(3, 4),  # 3=Holding, 4=Input
    cv.Optional(CONF_SCALE, default=0.1): cv.float_,
    cv.Optional(CONF_OFFSET, default=0.0): cv.float_,
    cv.Optional(CONF_UPDATE_INTERVAL, default="30s"): cv.update_interval,
    cv.Optional(CONF_UNIT_ID, default=1): cv.int_range(min=1, max=255),
    cv.Optional(CONF_REGISTERS_COUNT, default=1): cv.int_range(min=1, max=4),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_MODBUS_TCP_ID])
    
    var = cg.new_Pvariable(
        config[CONF_ID],
        parent,
        config[CONF_REGISTER_ADDRESS],
        config[CONF_FUNCTION_CODE],
        config[CONF_SCALE],
        config[CONF_OFFSET],
        config[CONF_UPDATE_INTERVAL].total_milliseconds,
        config[CONF_UNIT_ID],
        config[CONF_REGISTERS_COUNT]
    )
    
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)
