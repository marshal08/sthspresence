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
    // Initialize the sensor; no manual memory management
    if (!sth_.begin()) {
      ESP_LOGE(TAG, "Failed to initialize STHS34PF80!");
      this->mark_failed();
      return;
    }

    // Optional: configure ODR/filters if your driver supports them
    // sth_.setODR(STHS34PF80_ODR_1HZ);
    // sth_.enableBlockDataUpdate(true);

    ESP_LOGI(TAG, "STHS34PF80 initialized");
  }

  void update() override {
    // Read temperature with the correct method (object temperature).
    float temp_c = sth_.readObjectTemperature();

    // Sanity guard: if the driver returns nonsense, suppress it.
    bool temp_sane = !isnan(temp_c) && temp_c > -50.0f && temp_c < 120.0f;

    // Read presence and motion.
    // IMPORTANT: Replace the method names and types with the exact ones in your Adafruit_STHS34PF80.h.
    // If they return float or int16_t, adjust types accordingly.
    int presence = 0;
    int motion = 0;

    // Example placeholders — update to your driver’s actual methods:
    // presence = sth_.readPresence();
    // motion   = sth_.readMotion();

    // If your driver uses different names (e.g., readPresenceLevel(), readMotionDelta()),
    // change the calls above and consider scaling/clamping here.

    // Clamp counts (if applicable) to non-negative values
    if (presence < 0) presence = 0;
    if (motion < 0) motion = 0;

    // Publish safely
    if (temperature_sensor_) {
      temperature_sensor_->publish_state(temp_sane ? temp_c : NAN);
    }
    if (presence_sensor_) {
      presence_sensor_->publish_state(static_cast<float>(presence));
    }
    if (motion_sensor_) {
      motion_sensor_->publish_state(static_cast<float>(motion));
    }
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
