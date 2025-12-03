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

  void set_object_temperature_sensor(sensor::Sensor *s) { obj_temp_sensor_ = s; }
  void set_ambient_temperature_sensor(sensor::Sensor *s) { amb_temp_sensor_ = s; }
  void set_presence_sensor(sensor::Sensor *s) { presence_sensor_ = s; }
  void set_motion_sensor(sensor::Sensor *s) { motion_sensor_ = s; }

  void setup() override {
    if (!sth_.begin()) {
      ESP_LOGE(TAG, "Failed to initialize STHS34PF80!");
      this->mark_failed();
      return;
    }

    // Recommended basic configuration: block data update and a modest ODR
    // Adjust to your use-case; safeSetOutputDataRate exists internally in the driver.
    if (!sth_.setBlockDataUpdate(true)) {
      ESP_LOGW(TAG, "Failed to enable Block Data Update (BDU)");
    }
    if (!sth_.setOutputDataRate(STHS34PF80_ODR_1HZ)) {
      ESP_LOGW(TAG, "Failed to set ODR to 1 Hz");
    }

    // Optional: tune filters/averaging for stability
    // sth_.setTemperatureLowPassFilter(STHS34PF80_LPF_LOW);
    // sth_.setMotionLowPassFilter(STHS34PF80_LPF_MEDIUM);
    // sth_.setPresenceLowPassFilter(STHS34PF80_LPF_MEDIUM);
    // sth_.setAmbTempAveraging(STHS34PF80_AVG_T_T_8);
    // sth_.setObjAveraging(STHS34PF80_AVG_TMOS_8);

    ESP_LOGI(TAG, "STHS34PF80 initialized");
  }

  void update() override {
    // Respect data-ready if available to avoid stale/invalid reads.
    bool drdy = sth_.isDataReady();
    if (!drdy) {
      ESP_LOGD(TAG, "Data not ready yet");
      // Still publish NaN for temps to avoid stale values; presence/motion can be held.
      if (obj_temp_sensor_) obj_temp_sensor_->publish_state(NAN);
      if (amb_temp_sensor_) amb_temp_sensor_->publish_state(NAN);
      return;
    }

    // Read temperatures using provided API
    // Object temperature returns int16_t raw; compensated variant also int16_t.
    // Ambient temperature returns float.
    int16_t obj_raw = sth_.readObjectTemperature();                  // raw units (datasheet-defined scaling)
    float amb_c     = sth_.readAmbientTemperature();                  // already °C
    int16_t obj_comp = sth_.readCompensatedObjectTemperature();       // optional: may be closer to real °C scaling

    // Convert object temperature raw to °C if needed.
    // If your driver’s raw already represents 0.1°C steps or similar, apply conversion here.
    // For now, prefer compensated value when available; fall back to raw scaled heuristically if needed.
    float obj_c = NAN;
    if (obj_comp != 0) {
      // Heuristic: assume compensated value in 0.1°C steps (adjust if your driver documents different scaling)
      obj_c = static_cast<float>(obj_comp) / 10.0f;
    } else {
      // Fallback heuristic: scale raw similarly; update once you confirm exact scaling
      obj_c = static_cast<float>(obj_raw) / 10.0f;
    }

    // Sanity clamp temperatures
    auto sane_temp = [](float t) -> float {
      if (isnan(t)) return NAN;
      if (t < -50.0f || t > 120.0f) return NAN;
      return t;
    };
    obj_c = sane_temp(obj_c);
    amb_c = sane_temp(amb_c);

    // Read presence and motion (int16_t per your header)
    int16_t presence = sth_.readPresence();
    int16_t motion   = sth_.readMotion();

    // Clamp negative noise to zero; adjust scaling if needed later
    if (presence < 0) presence = 0;
    if (motion < 0) motion = 0;

    // Publish
    if (obj_temp_sensor_) obj_temp_sensor_->publish_state(obj_c);
    if (amb_temp_sensor_) amb_temp_sensor_->publish_state(amb_c);
    if (presence_sensor_) presence_sensor_->publish_state(static_cast<float>(presence));
    if (motion_sensor_) motion_sensor_->publish_state(static_cast<float>(motion));
  }

 protected:
    static constexpr const char *TAG = "sthspresence";
    Adafruit_STHS34PF80 sth_;
    sensor::Sensor *obj_temp_sensor_{nullptr};
    sensor::Sensor *amb_temp_sensor_{nullptr};
    sensor::Sensor *presence_sensor_{nullptr};
    sensor::Sensor *motion_sensor_{nullptr};
};

}  // namespace sthspresence
}  // namespace esphome
