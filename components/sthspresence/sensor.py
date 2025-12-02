import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_NAME

sthspresence_ns = cg.esphome_ns.namespace("sthspresence")
STHS34PF80Sensor = sthspresence_ns.class_("STHS34PF80Sensor", sensor.Sensor)

CONFIG_SCHEMA = sensor.sensor_schema(STHS34PF80Sensor).extend({
    cv.Required(CONF_NAME): cv.string,
})

def to_code(config):
    var = cg.new_Pvariable(config[CONF_NAME])
    yield sensor.register_sensor(var, config)
