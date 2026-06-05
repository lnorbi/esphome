#include "sprinklify_pressure_sensor.h"
#include "sprinklify_controller.h"  // included here, not in the header,
                                    // to avoid circular dependency
#include "esphome/core/log.h"
#include <algorithm>  // std::max, std::min
#include <cmath>      // std::isnan

namespace esphome {
namespace sprinklify {

static const char *const TAG = "sprinklify.pressure";

// ---------------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------------

void SprinklifyPressureSensor::set_calibration(float v_min, float v_max, float p_max, float opamp_at_5v) {
  this->cal_.v_min = v_min;
  this->cal_.v_max = v_max;
  this->cal_.p_max = p_max;
  // Precalculate everything we can here to save CPU in the hot path of every reading. The raw volts → bar conversion is
  // just a few math ops, but the OpAmp compensation involves a costly division, so we precompute the scale factor for
  // that.
  const float v_range = this->cal_.v_max - this->cal_.v_min;
  const float gain = this->cal_.p_max / v_range;
  this->cal_.opamp_scale = 5.0f / opamp_at_5v;
  this->cal_.adc_to_bar_scale = this->cal_.opamp_scale * gain;
  this->cal_.adc_to_bar_offset = -this->cal_.v_min * gain;
  this->cal_.valid = true;
  ESP_LOGD(TAG, "Calibration set: v_min=%.2f  v_max=%.2f  p_max=%.2f  opamp_scale=%.4f", this->cal_.v_min,
           this->cal_.v_max, this->cal_.p_max, this->cal_.opamp_scale);
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void SprinklifyPressureSensor::setup() {
  if (this->pressure_input_ == nullptr) {
    ESP_LOGE(TAG, "Pressure_input not set — pressure sensor will not function");
    this->mark_failed();
    return;
  }
  if (!this->cal_.valid) {
    ESP_LOGE(TAG, "Sensor calibration data not set - pressure sensor will not function");
    this->mark_failed();
    return;
  }

  this->pressure_input_->add_on_state_callback([this](float v) { this->on_raw_update_(v); });
  ESP_LOGD(TAG, "Pressure sensor ready — subscribed to raw input");
}

void SprinklifyPressureSensor::dump_config() {
  LOG_SENSOR("", "Sprinklify Pressure Sensor", this);
  ESP_LOGCONFIG(TAG, "  Calibration:");
  ESP_LOGCONFIG(TAG, "    V at 0 bar:    %.2f V", this->cal_.v_min);
  ESP_LOGCONFIG(TAG, "    V at p_max:    %.2f V", this->cal_.v_max);
  ESP_LOGCONFIG(TAG, "    p_max:         %.2f bar", this->cal_.p_max);
  ESP_LOGCONFIG(TAG, "    OpAmp scale:   %.4f", this->cal_.opamp_scale);
  ESP_LOGCONFIG(TAG, "  Stable threshold: %.3f bar/s", this->stable_threshold_);
  ESP_LOGCONFIG(TAG, "  Slope sensor:     %s", this->slope_sensor_ != nullptr ? "yes" : "no");
#ifdef USE_TEXT_SENSOR
  ESP_LOGCONFIG(TAG, "  Direction sensor: %s", this->direction_sensor_ != nullptr ? "yes" : "no");
#endif
}

// ---------------------------------------------------------------------------
// Signal chain
// ---------------------------------------------------------------------------

void SprinklifyPressureSensor::on_raw_update_(float adc_volts) {
  if (std::isnan(adc_volts)) {
    return;
  }
  const uint32_t now_ms = millis();

  // Step 1: calculate bar from volts using calibration constants
  const float bar = adc_volts * this->cal_.adc_to_bar_scale + this->cal_.adc_to_bar_offset;

  // Step 2: clamp to sensor's physical range
  const float clamped = std::clamp(bar, 0.0f, this->cal_.p_max);

  // Step 3: EMA smoothing — reduces ADC noise before slope/direction computation
  if (std::isnan(this->ema_bar_)) {
    this->ema_bar_ = clamped;  // seed with first real reading, no lag on startup
  } else {
    this->ema_bar_ = this->ema_alpha_ * clamped + (1.0f - this->ema_alpha_) * this->ema_bar_;
  }
  const float smoothed = this->ema_bar_;

  // Step 4: update slope and direction before publishing
  this->update_slope_(smoothed, now_ms);

  // Step 5: publish calibrated bar value to HA.
  this->publish_state(smoothed);

  // Step 6: notify hub — dry-run threshold comparison lives there, not here.
  // The hub owns the decision; this class owns the measurement.
  this->parent_->on_pressure_update(smoothed);
}

void SprinklifyPressureSensor::update_slope_(float bar, uint32_t now_ms) {
  if (std::isnan(this->last_bar_)) {
    // First reading — no previous value to diff against.
    // Store the baseline and leave direction as UNKNOWN.
    this->last_bar_ = bar;
    this->last_update_ms_ = now_ms;
    ESP_LOGV(TAG, "Slope: first reading (%.3f bar) — direction UNKNOWN", bar);
    return;
  }

  // Elapsed time in seconds using actual millis() diff.
  // This is always more accurate than the configured update_interval
  // because it reflects real scheduler timing, not the nominal interval.
  const uint32_t elapsed_ms = now_ms - this->last_update_ms_;

  if (elapsed_ms == 0) {
    // Defensive: avoid division by zero if two readings arrive simultaneously
    return;
  }

  const float elapsed_s = elapsed_ms / 1000.0f;
  const float slope = (bar - this->last_bar_) / elapsed_s;

  // Update state
  this->slope_bar_per_s_ = slope;
  this->last_bar_ = bar;
  this->last_update_ms_ = now_ms;

  // Classify direction using the configurable stability threshold
  PressureDirection new_direction;
  if (slope > this->stable_threshold_) {
    new_direction = PressureDirection::PRESSURE_RISING;
  } else if (slope < -this->stable_threshold_) {
    new_direction = PressureDirection::PRESSURE_FALLING;
  } else {
    new_direction = PressureDirection::PRESSURE_STABLE;
  }

  // Publish slope entity if wired — every reading
  if (this->slope_sensor_ != nullptr) {
    this->slope_sensor_->publish_state(slope);
  }

  // Publish direction entity only on change to avoid history spam
  if (new_direction != this->direction_) {
    this->direction_ = new_direction;
    // Notify controller of direction change — single authoritative source
    this->parent_->on_pressure_direction_changed(new_direction);

#ifdef USE_TEXT_SENSOR
    // Post to HA
    if (this->direction_sensor_ != nullptr) {
      this->direction_sensor_->publish_state(direction_to_str_(this->direction_));
    }
#endif
  }
  ESP_LOGV(TAG, "Pressure direction: %s (slope=%.3f bar/s)", direction_to_str_(this->direction_), slope);
}

const char *SprinklifyPressureSensor::direction_to_str_(PressureDirection d) {
  switch (d) {
    case PressureDirection::PRESSURE_RISING:
      return "rising";
    case PressureDirection::PRESSURE_FALLING:
      return "falling";
    case PressureDirection::PRESSURE_STABLE:
      return "stable";
    default:
      return "unknown";
  }
}

}  // namespace sprinklify
}  // namespace esphome
