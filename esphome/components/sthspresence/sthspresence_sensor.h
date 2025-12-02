// esphome/components/sthspresence/sthspresence_sensor.h
#pragma once

#include "esphome/core/component.h"   // PollingComponent
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"

#include "Adafruit_STHS34PF80.h"

namespace esphome {
namespace sthspresence {

class STHS34PF80Sensor : public PollingComponent {
 public:
  STHS34PF80Sensor() : PollingComponent(1000) {}

  void set_temperature_sensor(sensor::Sensor *s) { temperature_sensor_ = s; }
  void set_presence_sensor(sensor::Sensor *s) { presence_sensor_ = s; }
  void set_motion_sensor(sensor::Sensor *s) { motion_sensor_ = s; }

  void setup() override {
    if (!sth_.begin()) {
      ESP_LOGE(TAG, "Failed to initialize STHS34PF80!");
      this->mark_failed();
      return;
    }
    ESP_LOGI(TAG, "STHS34PF80 initialized");
  }

  void update() override {
    float temp = sth_.readTemperature();
    int presence = sth_.readPresence();
    int motion = sth_.readMotion();

    if (temperature_sensor_) temperature_sensor_->publish_state(temp);
    if (presence_sensor_) presence_sensor_->publish_state(presence);
    if (motion_sensor_) motion_sensor_->publish_state(motion);
  }

 protected:
  static constexpr const char *TAG = "sthspresence";
  Adafruit_STHS34PF80 sth_;
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *presence_sensor_{nullptr};
  sensor::Sensor *motion_sensor_{nullptr};
};

}  // namespace sthspresence
}  // namespace esphome

