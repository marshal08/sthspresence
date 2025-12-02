import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

sthspresence_ns = cg.esphome_ns.namespace("sthspresence")
STHS34PF80Sensor = sthspresence_ns.class_("STHS34PF80Sensor", cg.PollingComponent)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_ID): cv.declare_id(STHS34PF80Sensor),
    cv.Optional("temperature"): sensor.sensor_schema(),
    cv.Optional("presence"): sensor.sensor_schema(),
    cv.Optional("motion"): sensor.sensor_schema(),
})
