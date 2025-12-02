#include "esphome.h"
#include "Adafruit_STHS34PF80.h"

class STHS34PF80Sensor : public PollingComponent {
 public:
  Adafruit_STHS34PF80 sth;
  Sensor *temperature_sensor{nullptr};
  Sensor *presence_sensor{nullptr};
  Sensor *motion_sensor{nullptr};

  void set_temperature_sensor(Sensor *s) { temperature_sensor = s; }
  void set_presence_sensor(Sensor *s) { presence_sensor = s; }
  void set_motion_sensor(Sensor *s) { motion_sensor = s; }

  STHS34PF80Sensor() : PollingComponent(1000) {}

  void setup() override {
    if (!sth.begin()) {
      ESP_LOGE("sthspresence", "Failed to initialize STHS34PF80!");
    }
  }

  void update() override {
    sensors_event_t presence, motion, temp;
    sth.getEvent(&presence, &motion, &temp);

    if (presence_sensor) presence_sensor->publish_state(presence.data[0]);
    if (motion_sensor) motion_sensor->publish_state(motion.data[0]);
    if (temperature_sensor) temperature_sensor->publish_state(temp.temperature);
  }
};
