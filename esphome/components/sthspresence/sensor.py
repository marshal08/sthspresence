# esphome/components/sthspresence/sensor.py
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

# Namespace matches C++ namespace below
sthspresence_ns = cg.esphome_ns.namespace("sthspresence")
STHS34PF80Sensor = sthspresence_ns.class_("STHS34PF80Sensor", cg.PollingComponent)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_ID): cv.declare_id(STHS34PF80Sensor),
    cv.Optional("temperature"): sensor.sensor_schema(),
    cv.Optional("presence"): sensor.sensor_schema(),
    cv.Optional("motion"): sensor.sensor_schema(),
})

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)

    if "temperature" in config:
        sens = yield sensor.new_sensor(config["temperature"])
        cg.add(var.set_temperature_sensor(sens))

    if "presence" in config:
        sens = yield sensor.new_sensor(config["presence"])
        cg.add(var.set_presence_sensor(sens))

    if "motion" in config:
        sens = yield sensor.new_sensor(config["motion"])
        cg.add(var.set_motion_sensor(sens))

