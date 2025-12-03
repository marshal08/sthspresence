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

    // Basic recommended configuration
    if (!sth_.setBlockDataUpdate(true)) {
      ESP_LOGW(TAG, "Failed to enable Block Data Update (BDU)");
    }
    if (!sth_.setOutputDataRate(STHS34PF80_ODR_1_HZ)) {
      ESP_LOGW(TAG, "Failed to set ODR to 1 Hz");
    }

    // Optional: tune filters/averaging (uncomment if desired and supported)
    // (Choose configs from your driver’s enums)
    // sth_.setTemperatureLowPassFilter(STHS34PF80_LPF_MEDIUM);
    // sth_.setPresenceLowPassFilter(STHS34PF80_LPF_MEDIUM);
    // sth_.setMotionLowPassFilter(STHS34PF80_LPF_MEDIUM);
    // sth_.setAmbTempAveraging(STHS34PF80_AVG_T_T_8);
    // sth_.setObjAveraging(STHS34PF80_AVG_TMOS_8);
    // sth_.setWideGainMode(false);
    // sth_.setSensitivity(0);  // adjust to your scenario

    ESP_LOGI(TAG, "STHS34PF80 initialized");
  }

  void update() override {
    // Respect data-ready to avoid stale reads
    if (!sth_.isDataReady()) {
      ESP_LOGD(TAG, "Data not ready yet");
      // Publish NAN for temps when not ready to prevent stale values
      if (obj_temp_sensor_) obj_temp_sensor_->publish_state(NAN);
      if (amb_temp_sensor_) amb_temp_sensor_->publish_state(NAN);
      return;
    }

    // Ambient temperature: driver returns float °C
    float amb_c = sth_.readAmbientTemperature();

    // Object temperature: prefer compensated int16, fallback raw int16
    int16_t obj_comp_raw = sth_.readCompensatedObjectTemperature();
    int16_t obj_raw      = sth_.readObjectTemperature();

    // Heuristic scaling for int16 raw values; adjust if your .cpp documents differently
    // If compensated value is meaningful (non-zero or within plausible range), prefer it
    float obj_c = NAN;
    if (obj_comp_raw != 0) {
      obj_c = static_cast<float>(obj_comp_raw) / 10.0f;  // assume 0.1°C steps
    } else {
      obj_c = static_cast<float>(obj_raw) / 10.0f;       // fallback
    }

    // Sanity clamp temperatures
    auto sane_temp = [](float t) -> float {
      if (isnan(t)) return NAN;
      if (t < -50.0f || t > 120.0f) return NAN;
      return t;
    };
    amb_c = sane_temp(amb_c);
    obj_c = sane_temp(obj_c);

    // Presence and motion: int16_t raw amplitudes/counts
    int16_t presence = sth_.readPresence();
    int16_t motion   = sth_.readMotion();

    // Clamp negative noise to zero
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
