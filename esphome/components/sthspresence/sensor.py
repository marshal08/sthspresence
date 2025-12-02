import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    UNIT_CELSIUS,
    UNIT_EMPTY,
    ICON_THERMOMETER,
    CONF_ID,
)

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor"]

# Config keys
CONF_TEMPERATURE = "temperature"
CONF_PRESENCE = "presence"
CONF_MOTION = "motion"

# Namespace and class binding
sth_ns = cg.esphome_ns.namespace("sthspresence")
STHS34PF80Sensor = sth_ns.class_("STHS34PF80Sensor", cg.PollingComponent)

# Schema definition
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(STHS34PF80Sensor),

    cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        icon=ICON_THERMOMETER,
        accuracy_decimals=1,
    ),

    cv.Optional(CONF_PRESENCE): sensor.sensor_schema(
        unit_of_measurement=UNIT_EMPTY,
        icon="mdi:account",
        accuracy_decimals=1,
    ),

    cv.Optional(CONF_MOTION): sensor.sensor_schema(
        unit_of_measurement=UNIT_EMPTY,
        icon="mdi:run",
        accuracy_decimals=1,
    ),
})

# Code generation
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_PRESENCE in config:
        sens = await sensor.new_sensor(config[CONF_PRESENCE])
        cg.add(var.set_presence_sensor(sens))

    if CONF_MOTION in config:
        sens = await sensor.new_sensor(config[CONF_MOTION])
        cg.add(var.set_motion_sensor(sens))
