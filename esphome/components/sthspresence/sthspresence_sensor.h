// esphome/components/sthspresence/sthspresence_sensor.h
#pragma once

#include "esphome/core/log.h"
#include "esphome/components/polling_component/polling_component.h"
#include "esphome/components/sensor/sensor.h"

#include "Adafruit_STHS34PF80.h"
#include <Adafruit_Sensor.h>  // for sensors_event_t

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
    sensors_event_t presence_evt;
    sensors_event_t motion_evt;
    sensors_event_t temp_evt;

    // Adjust this call if your driver uses different names.
    // Many Adafruit Unified drivers expose getEvent(...)
    bool ok = sth_.getEvent(&presence_evt, &motion_evt, &temp_evt);
    if (!ok) {
      ESP_LOGW(TAG, "Failed to read events");
      return;
    }

    if (presence_sensor_) presence_sensor_->publish_state(presence_evt.data[0]);
    if (motion_sensor_) motion_sensor_->publish_state(motion_evt.data[0]);
    if (temperature_sensor_) temperature_sensor_->publish_state(temp_evt.temperature);
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

