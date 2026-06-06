#include "sprinklify_pressure_sensor.h"
#include "sprinklify_controller.h"
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
  ESP_LOGCONFIG(TAG, "  EMA alpha:         %.3f", this->ema_alpha_);
  ESP_LOGCONFIG(TAG, "  Slope window:      %u samples", this->slope_window_size_);
  ESP_LOGCONFIG(TAG, "  Stable threshold:  %.3f bar/s", this->stable_threshold_);
  ESP_LOGCONFIG(TAG, "  Dir. hysteresis:   %.3f bar/s", this->direction_hysteresis_);
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

  // Step 2: EMA smoothing — reduces ADC noise before slope/direction computation
  if (std::isnan(this->ema_bar_)) {
    this->ema_bar_ = bar;  // seed with first real reading, no lag on startup
  } else {
    this->ema_bar_ = this->ema_alpha_ * bar + (1.0f - this->ema_alpha_) * this->ema_bar_;
  }

  // Step 3: clamp to sensor's physical range
  const float clamped = std::clamp(this->ema_bar_, 0.0f, this->cal_.p_max);

  // Step 4: update slope and direction before publishing
  this->update_slope_(clamped, now_ms);

  // Step 5: publish calibrated bar value to HA. - Mind the recommended filters in YAML to throttle this
  this->publish_state(clamped);

  // Step 6: notify hub — dry-run threshold comparison lives there, not here.
  // The hub owns the decision; this class owns the measurement.
  this->parent_->on_pressure_update(clamped);
}

// ---------------------------------------------------------------------------
// Windowed slope + hysteretic direction
// ---------------------------------------------------------------------------

void SprinklifyPressureSensor::update_slope_(float bar, uint32_t now_ms) {
  // --- Write new sample into circular buffer ---
  this->slope_window_[this->slope_write_idx_] = {bar, now_ms};
  this->slope_write_idx_ = (this->slope_write_idx_ + 1) % this->slope_window_size_;
  if (this->slope_fill_count_ < this->slope_window_size_) {
    ++this->slope_fill_count_;
  }

  // Need at least 2 samples before we can compute a slope
  if (this->slope_fill_count_ < 2) {
    ESP_LOGV(TAG, "Slope: first reading (%.3f bar) — direction UNKNOWN", bar);
    return;
  }

  // --- Compute slope over the full window (newest minus oldest) ---
  // Write index now points one past the slot we just wrote, which is also the
  // oldest slot once the buffer is full (circular buffer invariant).
  // When not yet full, the oldest slot is index 0.

  // --- Collect window samples in chronological order ---
  // Start from the oldest slot and walk forward.
  const uint8_t n = this->slope_fill_count_;
  const uint8_t oldest_idx =
      (n < this->slope_window_size_) ? 0 : this->slope_write_idx_;  // write_idx already advanced past oldest

  const float t_origin_ms = static_cast<float>(this->slope_window_[oldest_idx].ms);

  float sum_t = 0.0f;   // Σt   (seconds, relative to oldest)
  float sum_y = 0.0f;   // Σy   (bar)
  float sum_tt = 0.0f;  // Σt²
  float sum_ty = 0.0f;  // Σ(t·y)

  for (uint8_t i = 0; i < n; ++i) {
    const uint8_t idx = (oldest_idx + i) % this->slope_window_size_;
    const float t = (static_cast<float>(this->slope_window_[idx].ms) - t_origin_ms) / 1000.0f;
    const float y = this->slope_window_[idx].bar;
    sum_t += t;
    sum_y += y;
    sum_tt += t * t;
    sum_ty += t * y;
  }

  const float fn = static_cast<float>(n);
  const float denom = fn * sum_tt - sum_t * sum_t;

  float slope = 0.0f;
  if (std::fabs(denom) > 1e-6f) {
    slope = (fn * sum_ty - sum_t * sum_y) / denom;  // bar/s
  }

  // --- Optional light EMA on slope output ---
  if (this->slope_ema_alpha_ < 1.0f) {
    if (std::isnan(this->slope_ema_)) {
      this->slope_ema_ = slope;
    } else {
      this->slope_ema_ += this->slope_ema_alpha_ * (slope - this->slope_ema_);
    }
    slope = this->slope_ema_;
  }
  this->slope_bar_per_s_ = slope;

  // Publish raw slope to diagnostic sensor every reading
  if (this->slope_sensor_ != nullptr) {
    this->slope_sensor_->publish_state(slope);
  }

  // Calculate direction changes
  this->update_direction_(slope);
}

void SprinklifyPressureSensor::update_direction_(float slope) {
  // --- Hysteretic direction candidate ---
  //
  // Entry  (leaving STABLE):  |slope| crosses stable_threshold_  outward
  // Exit   (back to STABLE):  |slope| drops below stable_threshold_ - direction_hysteresis_
  //
  // RISING ↔ FALLING routes through STABLE.

  PressureDirection candidate = this->direction_;
  const float inner = this->stable_threshold_ - this->direction_hysteresis_;

  switch (this->direction_) {
    case PressureDirection::UNKNOWN:
    case PressureDirection::PRESSURE_STABLE:
      if (slope > this->stable_threshold_)
        candidate = PressureDirection::PRESSURE_RISING;
      else if (slope < -this->stable_threshold_)
        candidate = PressureDirection::PRESSURE_FALLING;
      else
        candidate = PressureDirection::PRESSURE_STABLE;
      break;
    case PressureDirection::PRESSURE_RISING:
      if (slope < inner)
        candidate = PressureDirection::PRESSURE_STABLE;
      break;
    case PressureDirection::PRESSURE_FALLING:
      if (slope > -inner)
        candidate = PressureDirection::PRESSURE_STABLE;
      break;
  }

  // --- Debounce ---
  //
  // Required hold count depends on whether we are entering a directional
  // state or returning to STABLE.
  const bool exiting_to_stable = (candidate == PressureDirection::PRESSURE_STABLE);
  const uint8_t required = exiting_to_stable ? this->debounce_exit_count_ : this->debounce_enter_count_;

  if (candidate == this->direction_) {
    // Candidate matches committed direction — reset debounce state.
    this->pending_direction_ = this->direction_;
    this->debounce_count_ = 0;
  } else if (candidate == this->pending_direction_) {
    // Candidate is consistent with the pending change — advance counter.
    ++this->debounce_count_;
    if (this->debounce_count_ >= required) {
      // Held long enough — commit.
      this->direction_ = candidate;
      this->pending_direction_ = candidate;
      this->debounce_count_ = 0;
      this->parent_->on_pressure_direction_changed(this->direction_);
#ifdef USE_TEXT_SENSOR
      if (this->direction_sensor_ != nullptr)
        this->direction_sensor_->publish_state(direction_to_str_(this->direction_));
#endif
    }
  } else {
    // Candidate changed before the previous one was committed — start fresh.
    this->pending_direction_ = candidate;
    this->debounce_count_ = 1;
  }

  ESP_LOGV(TAG, "slope=%.4f  committed=%s  pending=%s  count=%u/%u", slope, direction_to_str_(this->direction_),
           direction_to_str_(this->pending_direction_), this->debounce_count_, required);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

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
