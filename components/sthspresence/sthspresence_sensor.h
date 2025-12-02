#pragma once
#include "esphome.h"
#include "Adafruit_STHS34PF80.h"

class STHS34PF80Sensor : public PollingComponent {
 public:
  Adafruit_STHS34PF80 sth;
  Sensor *temperature_sensor{nullptr};
  Sensor *presence_sensor{nullptr};
  Sensor *motion_sensor{nullptr};

  void set_temperature_sensor(Sensor *s);
  void set_presence_sensor(Sensor *s);
  void set_motion_sensor(Sensor *s);

  STHS34PF80Sensor();

  void setup() override;
  void update() override;
};
