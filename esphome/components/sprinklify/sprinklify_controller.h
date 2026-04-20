#pragma once

#include "esphome/core/component.h"
// TODO: Remove this! Temporary only!!
// #undef USE_TEXT_SENSOR
// #undef USE_BINARY_SENSOR
// #undef USE_NUMBER
// #undef USE_SWITCH
// -------------
#include "esphome/core/automation.h"
#include "esphome/core/preferences.h"
#include "esphome/core/helpers.h"
#include "esphome/components/output/binary_output.h"
#include "esphome/components/time/real_time_clock.h"

#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif

#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif

#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

#include "hsm.h"
#include "event_scheduler.h"
#include "sprinklify_pressure_sensor.h"
#include "sprinklify_flow_sensor.h"
#include "sprinklify_led_indicator.h"
#include "sprinklify_pump.h"

// Populated by codegen — equals the number of pumps declared in YAML.
// Used as the fixed array size throughout the controller.
#ifndef SPRINKLIFY_PUMP_COUNT
#define SPRINKLIFY_PUMP_COUNT 1  // NOLINT(cppcoreguidelines-macro-usage)
#endif

namespace esphome {
namespace sprinklify {

#define HSM_STATE_HANDLER_DECL(sname) StateResult state_##sname##_handler_(HSMEventType evt)
#define HSM_STATE_HANDLER(sname) StateResult SprinklifyController::state_##sname##_handler_(HSMEventType evt)
#define HSM_TRAN(state) (this->hsm_.transition_to(state))

static constexpr uint8_t NO_PUMP = 0xFF;  // Special pump index indicating "no pump"
static constexpr uint8_t PUMP_COUNT = SPRINKLIFY_PUMP_COUNT;

/// @brief Controller state machine events
enum Events : HSMEventType {
  EVT_SCHEDULER_TICK = HSM_FIRST_USER_EVENT,
  // External events
  EVT_MANUAL_START_REQUESTED,
  EVT_MANUAL_STOP_REQUESTED,
  EVT_UNBLOCK_SCHEDULE,
  EVT_MODE_CHANGE_AUTO,
  EVT_MODE_CHANGE_MANUAL,
  // Sensor events
  EVT_PRESSURE_LOW,
  EVT_PRESSURE_OK,
  EVT_PRESSURE_MAX,
  EVT_PRESSURE_FALLING,
  EVT_PRESSURE_STABLE,
  EVT_PRESSURE_RISING,
  EVT_FLOW_DETECTED,
  EVT_FLOW_LOST,
  // Pump events
  EVT_PUMP_STARTED,
  EVT_PUMP_STOPPED,

  // Internal events & timeouts
  EVT_PUMP_RESET,
  EVT_DRY_RUN_TIMEOUT_EXPIRED,
  EVT_NO_FLOW_SAFETY_TIMEOUT_EXPIRED,
  EVT_PUMP_MAX_RUNTIME_REACHED,
  EVT_INTERLOCK_EXPIRED,
  EVT_UNBLOCK_CYCLE_COMPLETE,
  EVT_SHUTDOWN,
};

class SprinklifyController : public Component {
#ifdef USE_BINARY_SENSOR
 public:
  enum class BinarySensorType {
    FLOW_OK = 0,
    PRESSURE_OK,
    BINARY_SENSOR_TYPE_COUNT,
  };
#endif

 public:
  void dump_config() override;
  void loop() override;
  void setup() override;
  void on_safe_shutdown() override;

  // -- Wiring setters — called from codegen before setup() ---
  void set_time(time::RealTimeClock *time) { this->time_id_ = time; }
  void set_pressure_sensor(SprinklifyPressureSensor *sens) { this->pressure_sensor_ = sens; }
  void set_flow_sensor(SprinklifyFlowSensor *sens) { this->flow_sensor_ = sens; }
  void set_min_flow(float flow) { this->min_flow_ = flow; }
  void set_no_flow_safety_timeout(uint32_t timeout_ms) { this->no_flow_safety_timeout_ms_ = timeout_ms; }
  void set_dry_run_timeout(uint32_t timeout_ms) { this->dry_run_timeout_ms_ = timeout_ms; }
  void set_empty_pressure(float press) { this->empty_pressure_ = press; }
  void set_max_pressure(float press) { this->max_pressure_ = press; }
  void set_pump_start_pressure(float press) { this->pump_start_pressure_ = press; }
  void set_interlock_delay(uint32_t delay_ms) { this->interlock_delay_ms_ = delay_ms; }
  void set_status_led_red_output(output::BinaryOutput *led_output) { this->status_led_red_.set_output(led_output); }
  void set_status_led_green_output(output::BinaryOutput *led_output) { this->status_led_green_.set_output(led_output); }
  void set_pressure_led_output(output::BinaryOutput *led_output) { this->pressure_led_.set_output(led_output); }
  void set_flow_led_output(output::BinaryOutput *led_output) { this->flow_led_.set_output(led_output); }
  void set_unblock_time(uint8_t hour, uint8_t minute, uint8_t second) {
    this->unblock_hour_ = hour;
    this->unblock_minute_ = minute;
    this->unblock_second_ = second;
  }

#ifdef USE_SWITCH
  void set_auto_mode_switch(switch_::Switch *sw) { this->auto_mode_switch_ = sw; }
  void set_winter_mode_switch(switch_::Switch *sw) { this->winter_mode_switch_ = sw; }
#endif

#ifdef USE_BINARY_SENSOR
  void set_binary_sensor(BinarySensorType type, binary_sensor::BinarySensor *sens);
#endif

#ifdef USE_NUMBER
  void set_pump_start_pressure_number(number::Number *n) { this->pump_start_pressure_number_ = n; }
  void set_max_pressure_number(number::Number *n) { this->max_pressure_number_ = n; }
  void set_pump_max_runtime_number(uint8_t i, number::Number *n) { pumps_[i].max_runtime_number = n; }
  void set_pump_auto_reset_wait_time_number(uint8_t i, number::Number *n) { pumps_[i].auto_reset_wait_time_number = n; }
#endif

#ifdef USE_TEXT_SENSOR
  void set_controller_state_sensor(text_sensor::TextSensor *sensor) { this->controller_state_sensor_ = sensor; }
  void set_pump_state_sensor(uint8_t i, text_sensor::TextSensor *sensor) { pumps_[i].state_sensor = sensor; }
#endif

  // Pump-level setters
  /// @brief Adds a pump to the controller with the given configuration. Pumps must be added in the same order as
  /// declared in YAML. The index is used to correlate the pump with its corresponding config and runtime state stored
  /// in fixed-size arrays.
  void add_pump(uint8_t index, output::BinaryOutput *relay, output::BinaryOutput *led, uint32_t max_runtime_ms,
                uint32_t auto_reset_wait_time_ms);
  void set_pump_total_runtime_sensor(uint8_t idx, sensor::Sensor *sens) {
    this->pumps_[idx].total_runtime_sensor = sens;
  }
  void set_pump_total_volume_sensor(uint8_t idx, sensor::Sensor *sens) { this->pumps_[idx].total_volume_sensor = sens; }

  // -- Callbacks — called by sensors and switches when their state changes ---
  void on_auto_mode_changed(bool val);
  void on_winter_mode_changed(bool val);
  void on_pressure_update(float press);
  void on_pressure_direction_changed(PressureDirection dir);
  void on_flow_update(float flow);
  void on_reset_pump(uint8_t pump_idx);

 protected:
  /// @brief Controller HSM states
  enum ControllerStates : HSMStateType {
    // clang-format off
    ROOT,
      OPERATIONAL,
        AUTO_MODE,
          AUTO_IDLE,
          AUTO_PUMPING,
            AUTO_PUMPING_WAITING_FOR_FLOW,
            AUTO_PUMPING_RUNNING,
          AUTO_FAULT,
        MANUAL_MODE,
          MANUAL_IDLE,
          MANUAL_RUNNING,
      UNBLOCK_ROUTINE,
        UNBLOCK_SINGLE_PUMP,
      INTERLOCK_WAIT,
    // clang-format on
  };

  // --- Switch & sensor accessors ---
  bool is_auto_mode_() const {
#ifdef USE_SWITCH
    return this->auto_mode_switch_ == nullptr || this->auto_mode_switch_->state;
#endif
    return true;
  }

  bool is_winter_mode_() const {
#ifdef USE_SWITCH
    return this->winter_mode_switch_ != nullptr && this->winter_mode_switch_->state;
#endif
    return false;
  }

  bool is_pressure_ok_() const {
    return std::isnan(this->pressure_) || this->pressure_ >= this->get_pump_start_pressure_();
  }

  bool is_flow_ok_() const {
    return this->flow_sensor_->has_state() && this->flow_sensor_->get_state() >= this->min_flow_;
  }

  float get_pump_start_pressure_() const {
#ifdef USE_NUMBER
    if (this->pump_start_pressure_number_ != nullptr && this->pump_start_pressure_number_->has_state())
      return this->pump_start_pressure_number_->state;
#endif
    return this->pump_start_pressure_;
  }

  float get_max_pressure_() const {
#ifdef USE_NUMBER
    if (this->max_pressure_number_ != nullptr && this->max_pressure_number_->has_state())
      return this->max_pressure_number_->state;
#endif
    return this->max_pressure_;
  }

  const char *state_as_str_(uint8_t state) {
    switch (state) {
      case ControllerStates::AUTO_IDLE:
        return "auto_idle";
      case ControllerStates::AUTO_PUMPING_WAITING_FOR_FLOW:
        return "auto_starting";
      case ControllerStates::AUTO_PUMPING_RUNNING:
        return "auto_running";
      case ControllerStates::AUTO_FAULT:
        return "auto_fault";
      case ControllerStates::MANUAL_IDLE:
        return "manual_idle";
      case ControllerStates::MANUAL_RUNNING:
        return "manual_running";
      case ControllerStates::INTERLOCK_WAIT:
        return "interlock";
      case ControllerStates::UNBLOCK_ROUTINE:
        return "unblock_routine";
      case STATE_INVALID:
        return "starting";
      default:
        return "unknown";
    }
  }

  /// --- Pump management ---
  bool start_next_pump_();
  void stop_active_pump_(bool faulted, bool latched);
  uint8_t get_first_available_pump_() const;

#ifdef USE_BINARY_SENSOR
  void update_binary_sensor_(BinarySensorType type, bool value);
#endif

  // --- Internal callbacks ---
  void led_tick_callback_();

  void publish_controller_state_();

  // State machine and state handlers
  HSM_STATE_HANDLER_DECL(root);
  HSM_STATE_HANDLER_DECL(operational);
  HSM_STATE_HANDLER_DECL(auto_mode);
  HSM_STATE_HANDLER_DECL(auto_idle);
  HSM_STATE_HANDLER_DECL(auto_pumping);
  HSM_STATE_HANDLER_DECL(auto_pumping_waiting_for_flow);
  HSM_STATE_HANDLER_DECL(auto_pumping_running);
  HSM_STATE_HANDLER_DECL(auto_fault);
  HSM_STATE_HANDLER_DECL(manual_mode);
  HSM_STATE_HANDLER_DECL(manual_idle);
  HSM_STATE_HANDLER_DECL(manual_running);
  HSM_STATE_HANDLER_DECL(unblock_routine);
  HSM_STATE_HANDLER_DECL(unblock_single_pump);
  HSM_STATE_HANDLER_DECL(interlock_wait);
  // HSM_STATE_HANDLER_DECL(unblock_complete);

  HSM hsm_{ControllerStates::ROOT};

  // Unblock routine
  uint8_t unblock_hour_{3};
  uint8_t unblock_minute_{0};
  uint8_t unblock_second_{0};
  uint16_t unblock_scheduled_event_{INVALID_EVENT_ID};

  // RTC management
  time::RealTimeClock *time_id_{nullptr};
  time_t now_{0};

  // Scheduler
  EventScheduler scheduler_;

  // Sensors and controls used by controller
  SprinklifyPressureSensor *pressure_sensor_{nullptr};
  SprinklifyFlowSensor *flow_sensor_{nullptr};
  // LEDs
  SprinklifyLEDIndicator status_led_red_;
  SprinklifyLEDIndicator status_led_green_;
  SprinklifyLEDIndicator pressure_led_;
  SprinklifyLEDIndicator flow_led_;

#ifdef USE_SWITCH
  switch_::Switch *auto_mode_switch_{nullptr};
  switch_::Switch *winter_mode_switch_{nullptr};
#endif

#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *binary_sensors_[(size_t) BinarySensorType::BINARY_SENSOR_TYPE_COUNT]{nullptr};
#endif

#ifdef USE_NUMBER
  number::Number *pump_start_pressure_number_{nullptr};
  number::Number *max_pressure_number_{nullptr};
#endif

#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *controller_state_sensor_{nullptr};
#endif

  /// Pumps and pump management
  Pump pumps_[PUMP_COUNT]{};  // All pumps in the system
  uint8_t active_pump_idx_{NO_PUMP};
  uint32_t interlock_delay_ms_{0};

  // Latest sensor readings and derived values
  float pressure_{NAN};
  float empty_pressure_{0.0f};
  float pump_start_pressure_{0.0f};
  float max_pressure_{0.0f};
  float min_flow_{0.0f};
  uint32_t no_flow_safety_timeout_ms_{0};
  uint32_t dry_run_timeout_ms_{0};
  float current_volume_liter_{0.0f};
  bool flow_ok_{false};
};

}  // namespace sprinklify
}  // namespace esphome
