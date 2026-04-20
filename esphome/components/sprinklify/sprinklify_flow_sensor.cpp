#include "sprinklify_flow_sensor.h"
#include "sprinklify_controller.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace sprinklify {

static const char *const TAG = "sprinklify.flow";
static constexpr uint32_t PUBLISH_INTERVAL_MS = 1000;

void SprinklifyFlowSensor::setup() {
  if (this->pulse_meter_ == nullptr) {
    ESP_LOGE(TAG, "Pulse_meter not set — flow sensor will not function");
    this->mark_failed();
    return;
  }
  // Force 1s timeout on the pulse meter — publishes 0 after 1s of no pulses
  this->pulse_meter_->set_timeout_us(PUBLISH_INTERVAL_MS * 1000);

  // Subscribe to raw pulse meter updates
  this->pulse_meter_->add_on_state_callback(
      [this](float pulses_per_min) { this->on_raw_pulse_update_(pulses_per_min); });
}

void SprinklifyFlowSensor::dump_config() {
  LOG_SENSOR("", "Sprinklify Flow Sensor", this);
  ESP_LOGCONFIG(TAG, "  Pulses per liter: %.1f", this->pulses_per_liter_);
}

void SprinklifyFlowSensor::loop() {
  const uint32_t now = millis();

  // Guarantee a report every second even if the pulse meter stays silent
  if (now - this->last_publish_ms_ >= PUBLISH_INTERVAL_MS) {
    this->last_publish_ms_ += PUBLISH_INTERVAL_MS;  // drift-resistant
    this->publish_state(this->last_flow_lpm_);
    this->parent_->on_flow_update(this->last_flow_lpm_);
  }
}

void SprinklifyFlowSensor::on_raw_pulse_update_(float pulses_per_min) {
  if (std::isnan(pulses_per_min)) {
    ESP_LOGW(TAG, "Pulse meter published NaN — skipping");
    return;
  }

  // Convert pulses/min → L/min
  this->last_flow_lpm_ = (this->pulses_per_liter_ > 0.0f) ? (pulses_per_min / this->pulses_per_liter_) : 0.0f;

  ESP_LOGV(TAG, "Raw: %.2f pulses/min → %.3f L/min", pulses_per_min, this->last_flow_lpm_);

  // Throttle: suppress immediate publish — loop() will pick it up within 1s
  // This avoids flooding the controller at high flow rates
}

}  // namespace sprinklify
}  // namespace esphome
