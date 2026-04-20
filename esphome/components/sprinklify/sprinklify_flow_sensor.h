#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/pulse_meter/pulse_meter_sensor.h"

namespace esphome {
namespace sprinklify {

class SprinklifyController;

class SprinklifyFlowSensor : public Component, public sensor::Sensor, public Parented<SprinklifyController> {
 public:
  void set_pulse_meter(pulse_meter::PulseMeterSensor *pulse_meter) { this->pulse_meter_ = pulse_meter; }
  void set_pulse_total_sensor(sensor::Sensor *pulse_total_sensor) { this->pulse_total_sensor_ = pulse_total_sensor; }
  void set_pulses_per_liter(float pulses_per_liter) { this->pulses_per_liter_ = pulses_per_liter; }

  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA - 1.0f; }

  void reset_total() { this->pulse_meter_->set_total_pulses(0); }
  float get_flow_lpm() const { return this->last_flow_lpm_; }
  float get_total_liters() const {
    return this->pulse_total_sensor_->has_state() ? this->pulse_total_sensor_->get_state() / this->pulses_per_liter_
                                                  : 0.0f;
  }

 private:
  pulse_meter::PulseMeterSensor *pulse_meter_{nullptr};
  sensor::Sensor *pulse_total_sensor_{nullptr};

  float pulses_per_liter_{1.0f};
  float last_flow_lpm_{0.0f};
  uint32_t last_publish_ms_{0};

  void on_raw_pulse_update_(float pulses_per_min);
};

}  // namespace sprinklify
}  // namespace esphome
