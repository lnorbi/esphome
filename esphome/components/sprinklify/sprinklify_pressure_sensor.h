#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome {
namespace sprinklify {

// Forward declaration — the hub is the parent; this class calls
// parent_->on_pressure_update_() but does not include the full hub header
// here to avoid a circular dependency. The .cpp includes it directly.
class SprinklifyController;

/// @brief Physical calibration constants for the pressure signal chain.
/// Populated once from codegen; immutable at runtime.
struct PressureCalibration {
  float v_min{0.5f};              // sensor output voltage at 0 bar
  float v_max{4.5f};              // sensor output voltage at p_max bar
  float p_max{10.0f};             // sensor full-scale pressure in bar
  float opamp_scale{1.0f};        // precomputed: opamp_scale = 5.0 / opamp_output_at_5v
  float adc_to_bar_scale{1.0f};   // precomputed: opamp_scale * (p_max / (v_max - v_min))
  float adc_to_bar_offset{0.0f};  // precomputed: -v_min * (p_max / (v_max - v_min))
  bool valid{false};              // whether calibration has been set
};

// ---------------------------------------------------------------------------
// Pressure trend direction — exposed to the hub for dry-run logic.
// ---------------------------------------------------------------------------
enum class PressureDirection : uint8_t {
  UNKNOWN,           // insufficient data (first reading, or after a long gap)
  PRESSURE_STABLE,   // rate of change within stable_threshold_
  PRESSURE_RISING,   // pressure increasing faster than stable_threshold_
  PRESSURE_FALLING,  // pressure decreasing faster than stable_threshold_
};

/// @brief Dedicated pressure sensor for Sprinklify.
///
/// Owns the complete signal chain from raw ADC volts to calibrated bar:
///
///   pressure_input (ADC sensor, volts)
///       ↓  on_raw_update_()
///   OpAmp correction  →  sensor linear map  →  clamp
///       ↓
///   publish_state()          — HA sees calibrated bar value
///       ↓
///   parent_->on_pressure_update()   — hub receives bar for dry-run logic
///
/// The hub is not involved in calibration. Swapping sensor types means
/// changing four numbers in YAML; no C++ or hub changes are required.
///
/// Lifecycle: setup() registers the ADC callback. Because this class
/// is a Component, ESPHome calls setup() before the first loop() tick,
/// guaranteeing the callback is in place before any readings arrive.
class SprinklifyPressureSensor : public Component, public sensor::Sensor, public Parented<SprinklifyController> {
 public:
  // --- Wiring setters — called from codegen before setup() ---
  void set_pressure_input(sensor::Sensor *input) { this->pressure_input_ = input; }
  void set_calibration(float v_min, float v_max, float p_max, float opamp_at_5v);
  void set_ema_alpha(float alpha) { this->ema_alpha_ = alpha; }
  void set_stable_threshold(float val) { this->stable_threshold_ = val; }
  void set_direction_hysteresis(float h) { this->direction_hysteresis_ = h; }
  void set_slope_window_size(uint8_t n) {
    // Clamp to compile-time buffer size
    this->slope_window_size_ = (n < 2) ? 2 : (n > MAX_SLOPE_WINDOW) ? MAX_SLOPE_WINDOW : n;
  }

  /// Optional EMA on the regression slope output. 1.0 = disabled.
  void set_slope_ema_alpha(float alpha) { this->slope_ema_alpha_ = alpha; }

  /// Consecutive samples a candidate direction must hold before being committed.
  /// Separate counts for entering a directional state (STABLE→RISING/FALLING or
  /// RISING→FALLING) vs returning to STABLE.
  void set_debounce_enter_count(uint8_t n) { this->debounce_enter_count_ = n; }
  void set_debounce_exit_count(uint8_t n) { this->debounce_exit_count_ = n; }

  // Optional slope and direction child entities
  void set_slope_sensor(sensor::Sensor *s) { this->slope_sensor_ = s; }
#ifdef USE_TEXT_SENSOR
  void set_direction_sensor(text_sensor::TextSensor *s) { this->direction_sensor_ = s; }
#endif

  // --- ESPHome lifecycle ---
  void setup() override;
  void dump_config() override;

  /// @brief Initialise after the hub (DATA priority) so that the hub's
  /// setup() and state machine start before the first pressure reading
  /// could theoretically arrive.
  float get_setup_priority() const override { return setup_priority::DATA - 1.0f; }

  // --- Controller accessors - read on demand
  float get_slope() const { return slope_bar_per_s_; }
  PressureDirection get_direction() const { return this->direction_; }
  bool is_pressure_rising() const { return this->direction_ == PressureDirection::PRESSURE_RISING; }
  bool is_pressure_falling() const { return this->direction_ == PressureDirection::PRESSURE_FALLING; }
  bool is_pressure_stable() const { return this->direction_ == PressureDirection::PRESSURE_STABLE; }

 private:
  // Maximum slope window depth.  Governs the circular buffer size at compile time.
  // 8 samples × 250 ms = 2 s maximum window; plenty of room for the default of 4.
  static constexpr uint8_t MAX_SLOPE_WINDOW = 8;

  // One slot in the slope circular buffer.
  struct SlopeSample {
    float bar{NAN};
    uint32_t ms{0};
  };

  /// @brief Invoked on every raw ADC reading. Applies calibration math,
  /// publishes the bar value to HA, and notifies the hub.
  void on_raw_update_(float adc_volts);

  /// @brief Updates the slope and direction state based on the new bar reading.
  ///
  /// Uses a circular window of the last slope_window_size_ samples to compute
  /// slope, giving inherent averaging over the window duration. Direction
  /// transitions are protected by hysteresis to prevent chatter near the
  /// stable_threshold_ boundary.
  void update_slope_(float bar, uint32_t now_ms);
  void update_direction_(float slope);

  /// @brief Converts a PressureDirection enum to a human-readable string for the direction text sensor.
  static const char *direction_to_str_(PressureDirection d);

  // --- Wiring ---
  sensor::Sensor *pressure_input_{nullptr};
  sensor::Sensor *slope_sensor_{nullptr};
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *direction_sensor_{nullptr};
#endif

  // --- Calibration constants (set once from codegen) ---
  PressureCalibration cal_;

  // --- EMA ---
  float ema_bar_{NAN};     // exponential moving average accumulator
  float ema_alpha_{0.1f};  // smoothing factor: 0=max smooth, 1=no smooth

  // --- Windowed slope ---
  // Circular buffer of the last N smoothed bar readings with their timestamps.
  // Slope = (newest.bar - oldest.bar) / elapsed_seconds.

  // Slope regression window
  std::array<SlopeSample, MAX_SLOPE_WINDOW> slope_window_{};
  uint8_t slope_write_idx_{0};    // next write position (circular)
  uint8_t slope_fill_count_{0};   // how many slots have been written (saturates at slope_window_size_)
  uint8_t slope_window_size_{6};  // number of samples in window (YAML: slope_window_size, default 6 = 1.5 s at 4 Hz)

  // Slope EMA (applied after regression)
  float slope_ema_alpha_{1.0f};  // 1.0 = pass-through
  float slope_ema_{NAN};

  float slope_bar_per_s_{NAN};

  // Direction thresholds
  // Threshold for STABLE classification (bar/s).  Transitions out of a direction
  // require the slope to fall below (threshold - hysteresis) before STABLE is declared.
  float stable_threshold_{0.05f};       // YAML: stable_threshold
  float direction_hysteresis_{0.015f};  // YAML: direction_hysteresis (default ~30% of threshold)

  // Committed direction — what has been reported to the controller
  PressureDirection direction_{PressureDirection::UNKNOWN};

  // Debounce state
  // A candidate direction must hold for the required count before direction_
  // is updated and the controller is notified.
  PressureDirection pending_direction_{PressureDirection::UNKNOWN};
  uint8_t debounce_count_{0};
  uint8_t debounce_enter_count_{3};  // samples to commit STABLE→RISING/FALLING (default 3 = 750 ms at 4 Hz)
  uint8_t debounce_exit_count_{2};   // samples to commit RISING/FALLING→STABLE (default 2 = 500 ms at 4 Hz)

  uint32_t last_update_ms_{0};  // millis() of the previous reading
};

}  // namespace sprinklify
}  // namespace esphome
