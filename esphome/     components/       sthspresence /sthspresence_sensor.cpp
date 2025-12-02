#include "sthspresence_sensor.h"

STHS34PF80Sensor::STHS34PF80Sensor() : PollingComponent(1000) {}

void STHS34PF80Sensor::set_temperature_sensor(Sensor *s) { temperature_sensor = s; }
void STHS34PF80Sensor::set_presence_sensor(Sensor *s) { presence_sensor = s; }
void STHS34PF80Sensor::set_motion_sensor(Sensor *s) { motion_sensor = s; }

void STHS34PF80Sensor::setup() {
  if (!sth.begin()) {
    ESP_LOGE("sthspresence", "Failed to initialize STHS34PF80!");
  }
}

void STHS34PF80Sensor::update() {
  sensors_event_t presence, motion, temp;
  sth.getEvent(&presence, &motion, &temp);

  if (presence_sensor) presence_sensor->publish_state(presence.data[0]);
  if (motion_sensor) motion_sensor->publish_state(motion.data[0]);
  if (temperature_sensor) temperature_sensor->publish_state(temp.temperature);
}


