#include "sprinklify_controller.h"

namespace esphome {
namespace sprinklify {

static const char *const TAG = "sprinklify";

constexpr uint32_t SCHEDULER_TICK_INTERVAL_MS = 1000;
constexpr uint32_t UNBLOCK_CYCLE_LEN_MS = 10 * 1000;  // 10 seconds
constexpr uint32_t LED_TICK_INTERVAL_MS = 100;
constexpr uint32_t STARTUP_DELAY_MS = 3000;

// Interval/timeout IDs (uint32_t to avoid string comparison)
enum : uint32_t {
  SCHEDULER_TICK_INTERVAL_ID = 0,
  LED_TICK_INTERVAL_ID,
  NO_FLOW_SAFETY_TIMEOUT_ID,
  DRY_RUN_TIMEOUT_ID,
  INTERLOCK_TIMEOUT_ID,
  PUMP_MAX_RUNTIME_TIMEOUT_ID,
  UNBLOCK_TIMEOUT_ID,
  START_TIMEOUT_ID,
};

// Helper macro to register a state with the HSM
#define HSM_REGISTER_STATE(state, handler_func, ...) \
  this->hsm_.register_state( \
      state, [this](HSMEventType evt) -> StateResult { return this->state_##handler_func##_handler_(evt); }, \
      ##__VA_ARGS__)

void SprinklifyController::dump_config() {
  ESP_LOGCONFIG(TAG, "Sprinklify Controller:");
  if (this->pressure_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Pressure sensor: %s", this->pressure_sensor_->get_name().c_str());
  }
  if (this->flow_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Flow sensor: %s", this->flow_sensor_->get_name().c_str());
  }
#ifdef USE_SWITCH
  if (this->auto_mode_switch_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Auto mode switch: %s", this->auto_mode_switch_->get_name().c_str());
  }
#endif

#ifdef USE_BINARY_SENSOR
  for (size_t i = 0; i < static_cast<size_t>(BinarySensorType::BINARY_SENSOR_TYPE_COUNT); ++i) {
    if (this->binary_sensors_[i] != nullptr) {
      ESP_LOGCONFIG(TAG, "  Binary sensor %u: %s", static_cast<unsigned>(i),
                    this->binary_sensors_[i]->get_name().c_str());
    }
  }
#endif
}

void SprinklifyController::setup() {
  // Setup LED timer
  this->set_interval(LED_TICK_INTERVAL_ID, LED_TICK_INTERVAL_MS, [this]() { this->led_tick_callback_(); });
  // Hook up master trigger callback
#ifdef USE_BINARY_SENSOR
  if (this->master_trigger_ != nullptr) {
    this->master_trigger_->add_on_state_callback([this](bool state) { this->on_master_trigger_changed(state); });
  }
#endif
  // Schedule daily unblock event if time is configured
  this->unblock_scheduled_event_ =
      this->scheduler_.add_daily_event(this->unblock_hour_, this->unblock_minute_, this->unblock_second_, [this]() {
        ESP_LOGI(TAG, "Unblock schedule triggered");
        this->hsm_.post_event(EVT_UNBLOCK_SCHEDULE);
      });

  // Configure pumps and load their persistent data
  for (size_t i = 0; i < PUMP_COUNT; i++) {
    auto *pump = &this->pumps_[i];
    pump->init_pref(i);
    if (!pump->load()) {
      ESP_LOGW(TAG, "Pump %d: failed to load persistent data — initialising fresh", i);
    } else {
      ESP_LOGI(TAG, "Pump %u: loaded — runtime=%" PRIu32 "s  volume=%.1fL  fault latched=%s", i,
               pump->persistent.total_runtime_s, pump->persistent.total_volume_l,
               pump->persistent.fault_latched ? "YES" : "no");
    }

    // Restore latched fault state from persistent data
    if (pump->persistent.fault_latched) {
      pump->set_status(PumpStatus::PUMP_FAULT);
      if (pump->persistent.last_fault_unix > 0 && pump->get_auto_reset_wait_time_ms() > 0) {
        const time_t reset_at = pump->persistent.last_fault_unix + pump->get_auto_reset_wait_time_ms() / 1000;
        pump->auto_reset_event_id =
            this->scheduler_.add_onetime_event(reset_at, [this, i]() { this->on_reset_pump(i); });
      }
    } else {
      pump->set_status(PumpStatus::PUMP_AVAILABLE);
    }
    pump->restore_sensors();
  }

  // Register HSM states
  bool res = true;
  // clang-format off
  res &= HSM_REGISTER_STATE(ControllerStates::ROOT, root);
  res &= HSM_REGISTER_STATE( ControllerStates::OPERATIONAL, operational, ControllerStates::ROOT);
  res &= HSM_REGISTER_STATE(   ControllerStates::AUTO_MODE, auto_mode, ControllerStates::OPERATIONAL);
  res &= HSM_REGISTER_STATE(     ControllerStates::AUTO_IDLE, auto_idle, ControllerStates::AUTO_MODE);
  res &= HSM_REGISTER_STATE(     ControllerStates::AUTO_PUMPING, auto_pumping, ControllerStates::AUTO_MODE);
  res &= HSM_REGISTER_STATE(       ControllerStates::AUTO_PUMPING_WAITING_FOR_FLOW, auto_pumping_waiting_for_flow, ControllerStates::AUTO_PUMPING);
  res &= HSM_REGISTER_STATE(       ControllerStates::AUTO_PUMPING_RUNNING, auto_pumping_running, ControllerStates::AUTO_PUMPING);
  res &= HSM_REGISTER_STATE(     ControllerStates::AUTO_FAULT, auto_fault, ControllerStates::AUTO_MODE);
  res &= HSM_REGISTER_STATE(   ControllerStates::MANUAL_MODE, manual_mode, ControllerStates::OPERATIONAL);
  res &= HSM_REGISTER_STATE(     ControllerStates::MANUAL_IDLE, manual_idle, ControllerStates::MANUAL_MODE);
  res &= HSM_REGISTER_STATE(     ControllerStates::MANUAL_RUNNING, manual_running, ControllerStates::MANUAL_MODE);
  res &= HSM_REGISTER_STATE( ControllerStates::UNBLOCK_ROUTINE, unblock_routine, ControllerStates::ROOT);
  res &= HSM_REGISTER_STATE(   ControllerStates::UNBLOCK_SINGLE_PUMP, unblock_single_pump, ControllerStates::UNBLOCK_ROUTINE);
  res &= HSM_REGISTER_STATE( ControllerStates::INTERLOCK_WAIT, interlock_wait, ControllerStates::ROOT);
  // clang-format on

  // Check for successful HSM setup
  if (!res) {
    ESP_LOGE(TAG, "HSM state registration error. Controller can't start.");
    return;
  }

  // We don't start HSM until pressure sensor has a valid reading. Setup start timeout.
  this->set_timeout(START_TIMEOUT_ID, STARTUP_DELAY_MS, [this]() {
    if (this->is_ready_for_start_()) {
      if (!this->hsm_.start()) {
        ESP_LOGE(TAG, "HSM not initialized properly. Controller can't start.");
      } else {
        ESP_LOGI(TAG, "Controller State Machine started.");
        // Re-evaluate master trigger — its initial state callback fired before
        // the HSM was ready and was silently dropped. If it is active right now,
        // we need to act on it.
#ifdef USE_BINARY_SENSOR
        if (this->master_trigger_ != nullptr && this->master_trigger_->has_state() && this->master_trigger_->state) {
          ESP_LOGI(TAG, "Master trigger was active at boot — posting deferred event");
          this->on_master_trigger_changed(true);
        }
#endif
      }
    } else {
      ESP_LOGE(TAG, "System not ready for start after %u ms. Check sensors.", STARTUP_DELAY_MS);
    }
  });
  ESP_LOGD(TAG, "Controller initialized. Waiting for HSM start.");
}

void SprinklifyController::on_safe_shutdown() {
  ESP_LOGI(TAG, "Safe shutdown — stopping all pumps");

  // Direct hardware de-energise first — unconditional safety guarantee
  for (uint8_t i = 0; i < PUMP_COUNT; i++) {
    if (this->pumps_[i].relay != nullptr)
      this->pumps_[i].relay->turn_off();
    this->pumps_[i].led.force_off();
  }
  this->status_led_red_.force_off();
  this->status_led_green_.force_off();

  // Attempt graceful HSM cleanup if running
  if (this->hsm_.is_ready()) {
    this->hsm_.post_event(EVT_SHUTDOWN);
    // HSM process only processes one event per call. We need a cycle to empty the queue
    while (!this->hsm_.is_idle()) {
      this->hsm_.process();  // synchronous drain — loop() won't fire again
    }
  }
}

/// @brief Main loop function for the controller. Processes HSM events.
void SprinklifyController::loop() { this->hsm_.process(); }

// ---------------------------------------------------------------------------
// Hierarchical State Machine (HSM) state handlers
// ---------------------------------------------------------------------------

/// @brief Root state handler
HSM_STATE_HANDLER(root) {
  // By default all events bubbling up to root will be considered handled, but just to be clear...
  StateResult res = RET_HANDLED;
  switch (evt) {
    case EVT_ENTRY:
      ESP_LOGI(TAG, "HSM started - root entry");
      // Arm scheduler tick event
      this->set_interval(SCHEDULER_TICK_INTERVAL_ID, SCHEDULER_TICK_INTERVAL_MS,
                         [this]() { this->hsm_.post_event(EVT_SCHEDULER_TICK); });
      break;
    case EVT_INIT:
      // Move on to operational mode
      res = HSM_TRAN(ControllerStates::OPERATIONAL);
      break;
    case EVT_SHUTDOWN:
      if (this->active_pump_idx_ != NO_PUMP) {
        auto *pump = &this->pumps_[this->active_pump_idx_];
        // Relay already off from on_safe_shutdown() direct call above
        // Just record stats and save
        const float vol = (this->flow_sensor_ != nullptr) ? this->flow_sensor_->get_total_liters() : 0.0f;
        pump->stop_run(vol);
        pump->save();
        this->active_pump_idx_ = NO_PUMP;
        ESP_LOGI(TAG, "Shutdown: pump stats saved");
      }
      break;
  }
  return res;
}

/// @brief Operational state handler
HSM_STATE_HANDLER(operational) {
  StateResult res = RET_UNHANDLED;
  switch (evt) {
    case EVT_ENTRY:
      ESP_LOGI(TAG, "HSM - operational state entry");
      res = RET_HANDLED;
      break;
    case EVT_EXIT:
      res = RET_HANDLED;
      break;

    case EVT_INIT:
      // Initial transition based on auto mode switch state
      if (this->is_auto_mode_()) {
        res = HSM_TRAN(ControllerStates::AUTO_MODE);
      } else {
        res = HSM_TRAN(ControllerStates::MANUAL_MODE);
      }
      break;

    case EVT_SCHEDULER_TICK: {
      ESPTime time = this->time_id_->now();
      if (time.is_valid()) {
        this->scheduler_.process_events(time);
        this->now_ = time.timestamp;
      }
      res = RET_HANDLED;
      break;
    }
    case EVT_UNBLOCK_SCHEDULE:
      // Event triggered by the daily scheduled event for pump unblock
      res = HSM_TRAN(ControllerStates::UNBLOCK_ROUTINE);
      break;
    // case EVT_PRESS_MAX:
    //   res = HSM_TRAN(ControllerStates::OPERATIONAL);
    //   break;
    case EVT_PUMP_STOPPED:
      // When a pump stops, we move over to interlock state
      res = HSM_TRAN(ControllerStates::INTERLOCK_WAIT);
      break;
  }
  return res;
}

HSM_STATE_HANDLER(auto_mode) {
  StateResult res = RET_UNHANDLED;
  switch (evt) {
    case EVT_ENTRY:
      // Make sure HA pump switches are all off
      reset_pump_switches_();
      res = RET_HANDLED;
      break;

    case EVT_EXIT:
      // This is fired when the auto mode switch is toggled off
      res = RET_HANDLED;
      break;

    case EVT_INIT:
      // Take initial transition to a sub-state
      if (this->is_winter_mode_()) {
        res = HSM_TRAN(ControllerStates::AUTO_WINTER);
      } else {
        if (this->is_pressure_ok_()) {
          res = HSM_TRAN(ControllerStates::AUTO_IDLE);
        } else {
          // Not enough pressure - need a pump
          res = HSM_TRAN(this->start_next_pump_() ? ControllerStates::AUTO_PUMPING : ControllerStates::AUTO_FAULT);
        }
      }
      break;

    case EVT_WINTER_MODE_ACTIVE:
      res = HSM_TRAN(ControllerStates::AUTO_WINTER);
      break;

    case EVT_WINTER_MODE_INACTIVE:
      // Back to auto mode. Transition-to-Self pattern guarantees that initial transition is taken.
      res = HSM_TRAN(ControllerStates::AUTO_MODE);
      break;

    case EVT_MODE_CHANGE_MANUAL:
      // Move to manual state
      res = HSM_TRAN(ControllerStates::MANUAL_MODE);
      break;
  }
  return res;
}

HSM_STATE_HANDLER(auto_idle) {
  StateResult res = RET_UNHANDLED;
  switch (evt) {
    case EVT_ENTRY:
      // Re-evaluate master trigger — its edge may have arrived during unblock
      // and been swallowed. If it's currently active, act on it now.
#ifdef USE_BINARY_SENSOR
      if (this->master_trigger_ != nullptr && this->master_trigger_->state) {
        this->hsm_.post_event(EVT_MASTER_TRIGGER_ACTIVE);
      }
#endif
      this->publish_controller_state_();
      res = RET_HANDLED;
      break;
    case EVT_EXIT:
      res = RET_HANDLED;
      break;

    case EVT_PRESSURE_LOW:
      // Find and start first available pump
      res = HSM_TRAN(this->start_next_pump_() ? ControllerStates::AUTO_PUMPING : ControllerStates::AUTO_FAULT);
      break;

    case EVT_MASTER_TRIGGER_ACTIVE:
      res = HSM_TRAN(this->start_next_pump_() ? ControllerStates::AUTO_PUMPING : ControllerStates::AUTO_FAULT);
      break;
  }
  return res;
}

HSM_STATE_HANDLER(auto_pumping) {
  StateResult res = RET_UNHANDLED;
  switch (evt) {
    case EVT_ENTRY:
      // Set up max runtime timeout
      this->set_timeout(PUMP_MAX_RUNTIME_TIMEOUT_ID, this->pumps_[this->active_pump_idx_].get_max_runtime_ms(),
                        [this]() { this->hsm_.post_event(EVT_PUMP_MAX_RUNTIME_REACHED); });

      // Reset flow sensor's total counter
      this->publish_controller_state_();
      res = RET_HANDLED;
      break;
    case EVT_EXIT:
      // If we're forced to exit (e.g. due to switch to manual mode), make sure pump is off
      this->stop_active_pump_(true, true);
      this->cancel_timeout(PUMP_MAX_RUNTIME_TIMEOUT_ID);
      res = RET_HANDLED;
      break;

    case EVT_INIT:
      // Check for flow here - we might not get another flow_detected event if it was already detected
      res = HSM_TRAN(this->is_flow_ok_() ? ControllerStates::AUTO_PUMPING_RUNNING
                                         : ControllerStates::AUTO_PUMPING_WAITING_FOR_FLOW);
      break;

    case EVT_PUMP_MAX_RUNTIME_REACHED:
      // Pump ran too long, stop, mark it as fault
      ESP_LOGD(TAG, "Max runtime reached for pump %" PRIu8, this->active_pump_idx_);
      this->stop_active_pump_(true, true);
      // No transition here - the EVT_PUMP_STOPPED event triggered by turning off the pump will move us to interlock
      // state, and from there we will decide if we can try another pump or need to move to fault state.
      res = RET_HANDLED;
      break;

    case EVT_WINTER_MODE_ACTIVE:
    case EVT_MASTER_TRIGGER_INACTIVE:
    case EVT_MODE_CHANGE_MANUAL:
      this->stop_active_pump_(false, false);
      // EVT_PUMP_STOPPED → INTERLOCK_WAIT as normal
      res = RET_HANDLED;
      break;
  }
  return res;
}

HSM_STATE_HANDLER(auto_pumping_waiting_for_flow) {
  StateResult res = RET_UNHANDLED;
  switch (evt) {
    case EVT_ENTRY:
      // Arm dry run protection timeout - we wait at most this long for the pump to start producing flow
      this->set_timeout(NO_FLOW_SAFETY_TIMEOUT_ID, this->no_flow_safety_timeout_ms_,
                        [this]() { this->hsm_.post_event(EVT_NO_FLOW_SAFETY_TIMEOUT_EXPIRED); });
      // Check if pressure was already falling or stable on entry - arm dry run time accordingly
      if (this->pressure_sensor_->get_direction() != PressureDirection::PRESSURE_RISING) {
        // Pressure dropping — could be dry run, give it flow_timeout to confirm
        this->set_timeout(DRY_RUN_TIMEOUT_ID, this->dry_run_timeout_ms_,
                          [this]() { this->hsm_.post_event(EVT_DRY_RUN_TIMEOUT_EXPIRED); });
      }
      this->publish_controller_state_();
      res = RET_HANDLED;
      break;
    case EVT_EXIT:
      this->cancel_timeout(NO_FLOW_SAFETY_TIMEOUT_ID);
      this->cancel_timeout(DRY_RUN_TIMEOUT_ID);
      res = RET_HANDLED;
      break;

    case EVT_FLOW_DETECTED:
      res = HSM_TRAN(ControllerStates::AUTO_PUMPING_RUNNING);
      break;

    case EVT_PRESSURE_FALLING:
    case EVT_PRESSURE_STABLE:
      // Pressure dropping or stabilized — could be dry run, give it dry_run_timeout to confirm
      this->set_timeout(DRY_RUN_TIMEOUT_ID, this->dry_run_timeout_ms_,
                        [this]() { this->hsm_.post_event(EVT_DRY_RUN_TIMEOUT_EXPIRED); });
      res = RET_HANDLED;
      break;
    case EVT_PRESSURE_RISING:
      // Pump is able to generate pressure, definitely not dry run - cancel dry run timeout
      this->cancel_timeout(DRY_RUN_TIMEOUT_ID);
      res = RET_HANDLED;
      break;
    case EVT_PRESSURE_MAX:
    case EVT_DRY_RUN_TIMEOUT_EXPIRED:
      // Pump will stop. Check exit condition and decide whether pump must be marked faulted.
      this->stop_active_pump_(/*faulted=*/!this->is_pressure_ok_(), false);
      // No transition here - the EVT_PUMP_STOPPED event triggered by turning off the pump will move us to interlock
      // state, and from there we will decide if we can try another pump or need to move to fault state.
      res = RET_HANDLED;
      break;
    case EVT_NO_FLOW_SAFETY_TIMEOUT_EXPIRED:
      // Absolute ceiling reached — something is wrong, safety shutdown
      ESP_LOGD(TAG, "Pump %d: start max timeout — safety fault", this->active_pump_idx_);
      this->stop_active_pump_(/*faulted=*/true, false);
      // No transition here - the EVT_PUMP_STOPPED event triggered by turning off the pump will move us to interlock
      // state, and from there we will decide if we can try another pump or need to move to fault state.
      return RET_HANDLED;
      break;
  }
  return res;
}

HSM_STATE_HANDLER(auto_pumping_running) {
  StateResult res = RET_UNHANDLED;
  switch (evt) {
    case EVT_ENTRY:
      this->publish_controller_state_();
      res = RET_HANDLED;
      break;
    case EVT_EXIT:
      res = RET_HANDLED;
      break;
    case EVT_FLOW_LOST:
      // Flow lost - don't immediately kill the pump, this might be temporary. Move back to waiting_for_flow state.
      res = HSM_TRAN(ControllerStates::AUTO_PUMPING_WAITING_FOR_FLOW);
      break;
      // case EVT_PUMP_MAX_RUNTIME_REACHED:
      //   auto *pump = &this->pumps_[this->active_pump_idx_];
      //   pump->turn_off();
      //   pump->record_fault(false, 0);
      //   // Turning a pump off will trigger an EVT_PUMP_STOPPED, which will be handled by the OPERATIONAL superstate.
      //   // It will transition the HSM to the INTERLOCK state
      //   res = RET_HANDLED;
      //   break;
  }
  return res;
}

/// @brief Auto - fault state handler
HSM_STATE_HANDLER(auto_fault) {
  StateResult res = RET_UNHANDLED;
  switch (evt) {
    case EVT_ENTRY:
      this->publish_controller_state_();
      res = RET_HANDLED;
      break;
    case EVT_PUMP_RESET:
      res = HSM_TRAN(ControllerStates::OPERATIONAL);
      break;
  }
  return res;
}

/// @brief Auto - fault state handler
HSM_STATE_HANDLER(auto_winter) {
  StateResult res = RET_UNHANDLED;
  switch (evt) {
    case EVT_ENTRY:
      this->publish_controller_state_();
      res = RET_HANDLED;
      break;
  }
  return res;
}

HSM_STATE_HANDLER(manual_mode) {
  StateResult res = RET_UNHANDLED;
  switch (evt) {
    case EVT_ENTRY:
      // Publish switch OFF for all pump run switches to sync HA with idle reality
      reset_pump_switches_();
      res = StateResult::RET_HANDLED;
      break;
    case EVT_EXIT:
      // Publish switch OFF for all pump run switches to sync HA with idle reality
      reset_pump_switches_();
      res = StateResult::RET_HANDLED;
      break;
    case EVT_INIT:
      // Always start in IDLE state here
      res = HSM_TRAN(ControllerStates::MANUAL_IDLE);
      break;
    case EVT_MODE_CHANGE_AUTO:
      res = HSM_TRAN(ControllerStates::AUTO_MODE);
      break;
  }
  return res;
}

HSM_STATE_HANDLER(manual_idle) {
  StateResult res = RET_UNHANDLED;
  switch (evt) {
    case EVT_ENTRY:
      this->publish_controller_state_();
      res = StateResult::RET_HANDLED;
      break;
    case EVT_EXIT:
      res = StateResult::RET_HANDLED;
      break;
    case EVT_MANUAL_START_REQUESTED:
      if (this->requested_pump_idx_ == NO_PUMP) {
        ESP_LOGW(TAG, "Manual start: no pump index set — ignoring");
        res = RET_HANDLED;
        break;
      }
      if (!this->start_specific_pump_(this->requested_pump_idx_)) {
        // Pump unavailable (faulted, not installed, etc.) — push switch back to OFF
#ifdef USE_SWITCH
        if (this->pumps_[this->requested_pump_idx_].run_switch != nullptr)
          this->pumps_[this->requested_pump_idx_].run_switch->sync_state(false);
#endif
        this->requested_pump_idx_ = NO_PUMP;
        res = RET_HANDLED;
        break;
      }
      res = HSM_TRAN(ControllerStates::MANUAL_RUNNING);
      break;
  }
  return res;
}

HSM_STATE_HANDLER(manual_running) {
  StateResult res = RET_UNHANDLED;
  switch (evt) {
    case EVT_ENTRY:
      // Arm absolute no-flow ceiling (same as auto mode)
      this->set_timeout(NO_FLOW_SAFETY_TIMEOUT_ID, this->no_flow_safety_timeout_ms_,
                        [this]() { this->hsm_.post_event(EVT_NO_FLOW_SAFETY_TIMEOUT_EXPIRED); });
      // Arm max runtime (same mechanism as auto mode)
      this->set_timeout(PUMP_MAX_RUNTIME_TIMEOUT_ID, this->pumps_[this->active_pump_idx_].get_max_runtime_ms(),
                        [this]() { this->hsm_.post_event(EVT_PUMP_MAX_RUNTIME_REACHED); });
      this->publish_controller_state_();
      res = StateResult::RET_HANDLED;
      break;

    case EVT_EXIT:
      this->cancel_timeout(NO_FLOW_SAFETY_TIMEOUT_ID);
      this->cancel_timeout(PUMP_MAX_RUNTIME_TIMEOUT_ID);
      res = StateResult::RET_HANDLED;
      break;

    case EVT_FLOW_DETECTED:
      // Flow confirmed — cancel dry-run timer. No-flow safety stays armed.
      this->cancel_timeout(DRY_RUN_TIMEOUT_ID);
      res = RET_HANDLED;
      break;

    case EVT_FLOW_LOST:
      // Flow lost — re-arm dry-run timer. This mirrors auto mode's WaitingForFlow behaviour.
      this->set_timeout(DRY_RUN_TIMEOUT_ID, this->dry_run_timeout_ms_,
                        [this]() { this->hsm_.post_event(EVT_DRY_RUN_TIMEOUT_EXPIRED); });
      res = RET_HANDLED;
      break;

    case EVT_NO_FLOW_SAFETY_TIMEOUT_EXPIRED:
      ESP_LOGW(TAG, "Manual pump %" PRIu8 ": no-flow safety timeout — safety fault", this->active_pump_idx_);
      this->stop_active_pump_(/*faulted=*/true, /*latched=*/false);
      res = RET_HANDLED;
      break;

    case EVT_PUMP_MAX_RUNTIME_REACHED:
      ESP_LOGW(TAG, "Manual pump %" PRIu8 ": max runtime reached — stopping with fault", this->active_pump_idx_);
      this->stop_active_pump_(/*faulted=*/true, /*latched=*/true);
      res = RET_HANDLED;
      break;

    case EVT_MANUAL_STOP_REQUESTED:
      // Operator or HA requested stop — clean, no fault
      ESP_LOGI(TAG, "Manual pump %" PRIu8 ": stop requested", this->active_pump_idx_);
      this->stop_active_pump_(/*faulted=*/false, /*latched=*/false);
      // EVT_PUMP_STOPPED → INTERLOCK_WAIT → back to MANUAL_IDLE
      res = RET_HANDLED;
      break;

    case EVT_MODE_CHANGE_AUTO:
      // Moving back to auto mode - stop the active pump here
      ESP_LOGI(TAG, "Change to auto mode requested. Stopping pump %" PRIu8, this->active_pump_idx_);
      this->stop_active_pump_(/*faulted=*/false, /*latched=*/false);
      // EVT_PUMP_STOPPED → INTERLOCK_WAIT → back to MANUAL_IDLE
      res = RET_HANDLED;
      break;

    // Pressure events — explicitly ignored in manual mode.
    // They bubble up to auto_mode and operational but neither handles them
    // while we're in MANUAL_RUNNING, so return RET_HANDLED here to stop bubbling.
    case EVT_PRESSURE_LOW:
    case EVT_PRESSURE_OK:
    case EVT_PRESSURE_MAX:
    case EVT_PRESSURE_RISING:
    case EVT_PRESSURE_STABLE:
    case EVT_PRESSURE_FALLING:
      res = RET_HANDLED;
      break;
  }
  return res;
}

HSM_STATE_HANDLER(interlock_wait) {
  // Intentionally ignore any events received while in interlock
  StateResult res = RET_IGNORED;
  switch (evt) {
    case EVT_ENTRY:
      // Arm interlock delay
      this->set_timeout(INTERLOCK_TIMEOUT_ID, this->interlock_delay_ms_,
                        [this]() { this->hsm_.post_event(Events::EVT_INTERLOCK_EXPIRED); });
      this->publish_controller_state_();
      res = StateResult::RET_HANDLED;
      break;
    case EVT_EXIT:
      res = StateResult::RET_HANDLED;
      break;
    case EVT_INTERLOCK_EXPIRED:
      // Move back to operational state
      res = HSM_TRAN(ControllerStates::OPERATIONAL);
      break;
  }
  return res;
}

HSM_STATE_HANDLER(unblock_routine) {
  StateResult res = RET_UNHANDLED;
  switch (evt) {
    case EVT_ENTRY:
      ESP_LOGD(TAG, "Starting unblock routine");
      this->active_pump_idx_ = 0;  // Start unblock cycle with the first pump
      this->publish_controller_state_();
      res = RET_HANDLED;
      break;
    case EVT_EXIT:
      this->active_pump_idx_ = NO_PUMP;  // Clear active pump index when leaving unblock routine, just in case
      res = RET_HANDLED;
      break;
    case EVT_INIT:
      res = HSM_TRAN(ControllerStates::UNBLOCK_SINGLE_PUMP);
      break;
  }
  return res;
}

HSM_STATE_HANDLER(unblock_single_pump) {
  StateResult res = RET_UNHANDLED;
  switch (evt) {
    case EVT_ENTRY:
      this->pumps_[this->active_pump_idx_].relay->turn_on();
      this->set_timeout(UNBLOCK_TIMEOUT_ID, UNBLOCK_CYCLE_LEN_MS,
                        [this]() { this->hsm_.post_event(EVT_UNBLOCK_CYCLE_COMPLETE); });
      res = RET_HANDLED;
      break;
    case EVT_EXIT:
      this->cancel_timeout(UNBLOCK_TIMEOUT_ID);
      res = RET_HANDLED;
      break;
    case EVT_UNBLOCK_CYCLE_COMPLETE:
      this->pumps_[this->active_pump_idx_].relay->turn_off();
      // Unblock or not, we still need to respect interlock delay
      this->set_timeout(INTERLOCK_TIMEOUT_ID, this->interlock_delay_ms_,
                        [this]() { this->hsm_.post_event(Events::EVT_INTERLOCK_EXPIRED); });
      res = RET_HANDLED;
      break;

    case EVT_INTERLOCK_EXPIRED:
      // Move on to next pump or exit routine if all pumps done
      if (this->active_pump_idx_ + 1 < PUMP_COUNT) {
        ++this->active_pump_idx_;
        // Transition to self will exit / re-enter the state, triggering the next pump cycle
        res = HSM_TRAN(ControllerStates::UNBLOCK_SINGLE_PUMP);
      } else {
        res = HSM_TRAN(ControllerStates::OPERATIONAL);
      }
      break;
  }
  return res;
}

// ---------------------------------------------------------------------------
// Codegen helper functions
// ---------------------------------------------------------------------------

/// @brief Adds a pump to the controller with the given configuration. Pumps must be added in the same order as
/// declared in YAML. The index is used to correlate the pump with its corresponding config and runtime state stored
/// in fixed-size arrays.
void SprinklifyController::add_pump(uint8_t index, output::BinaryOutput *relay, output::BinaryOutput *led,
                                    uint32_t max_runtime_ms, uint32_t auto_reset_wait_time_ms) {
  this->pumps_[index].relay = relay;
  this->pumps_[index].led.set_output(led);
  this->pumps_[index].max_runtime_ms = max_runtime_ms;
  this->pumps_[index].auto_reset_wait_time_ms = auto_reset_wait_time_ms;
}

#ifdef USE_BINARY_SENSOR
void SprinklifyController::set_binary_sensor(BinarySensorType type, binary_sensor::BinarySensor *sens) {
  if (type < BinarySensorType::BINARY_SENSOR_TYPE_COUNT) {
    this->binary_sensors_[(size_t) type] = sens;
  }
}
#endif

// ---------------------------------------------------------------------------
// Callbacks for sensors/switches to report changes to the controller
// ---------------------------------------------------------------------------

void SprinklifyController::on_pressure_update(float press) {
  // Value is guaranteed valid — SprinklifyPressureSensor filtered NaN upstream
  this->pressure_ = press;
  // Update LED and sensors
  this->pressure_led_.set_pattern(this->is_pressure_ok_() ? SprinklifyLEDIndicator::PATTERN_SOLID_ON
                                                          : SprinklifyLEDIndicator::PATTERN_OFF);
#ifdef USE_BINARY_SENSOR
  this->update_binary_sensor_(BinarySensorType::PRESSURE_OK, this->is_pressure_ok_());
#endif

  ESP_LOGV(TAG, "New pressure reported: %.1f bar", press);

  // Stop here if HSM is not ready yet
  if (!this->hsm_.is_ready()) {
    return;
  }

  // Pump needed event - always fire
  this->hsm_.post_event(this->is_pressure_ok_() ? EVT_PRESSURE_OK : EVT_PRESSURE_LOW);
  // Check for max system pressure and fire event
  if (this->pressure_ >= this->get_max_pressure_()) {
    this->hsm_.post_event(EVT_PRESSURE_MAX);
  }
}

void SprinklifyController::on_pressure_direction_changed(PressureDirection dir) {
  if (!this->hsm_.is_ready()) {
    return;
  }
  switch (dir) {
    case PressureDirection::PRESSURE_RISING:
      this->hsm_.post_event(EVT_PRESSURE_RISING);
      break;
    case PressureDirection::PRESSURE_STABLE:
      this->hsm_.post_event(EVT_PRESSURE_STABLE);
      break;
    case PressureDirection::PRESSURE_FALLING:
      this->hsm_.post_event(EVT_PRESSURE_FALLING);
      break;
  }
}

void SprinklifyController::on_flow_update(float flow) {
  this->flow_ = flow;
  this->flow_ok_ = flow > this->min_flow_;
  // Update sensors & LED
  this->flow_led_.set_pattern(this->flow_ok_ ? SprinklifyLEDIndicator::PATTERN_SOLID_ON
                                             : SprinklifyLEDIndicator::PATTERN_OFF);
#ifdef USE_BINARY_SENSOR
  this->update_binary_sensor_(BinarySensorType::FLOW_OK, this->flow_ok_);
#endif
  // Also post event if HSM is ready
  if (!this->hsm_.is_ready()) {
    return;
  }
  this->hsm_.post_event(this->flow_ok_ ? EVT_FLOW_DETECTED : EVT_FLOW_LOST);
}

void SprinklifyController::on_auto_mode_changed(bool val) {
  if (!this->hsm_.is_ready()) {
    return;
  }
  ESP_LOGI(TAG, "Mode change triggered to %s mode.", val ? "AUTO" : "MANUAL");
  this->hsm_.post_event(val ? EVT_MODE_CHANGE_AUTO : EVT_MODE_CHANGE_MANUAL);
}

void SprinklifyController::on_winter_mode_changed(bool val) {
  if (!this->hsm_.is_ready()) {
    return;
  }
  ESP_LOGI(TAG, "Winted mode change triggered to %s mode.", val ? "AUTO" : "MANUAL");
  this->hsm_.post_event(val ? EVT_WINTER_MODE_ACTIVE : EVT_WINTER_MODE_INACTIVE);
}

void SprinklifyController::on_reset_pump(uint8_t pump_idx) {
  if (!this->pumps_[pump_idx].is_faulted()) {
    ESP_LOGW(TAG, "Attempted to reset pump %" PRIu8 " which is not in fault state", pump_idx);
    return;
  }
  this->scheduler_.remove_event(this->pumps_[pump_idx].auto_reset_event_id);
  this->pumps_[pump_idx].reset();  // Note: this call must be second, as it invalidates the auto_reset_event_id
  ESP_LOGI(TAG, "Pump %" PRIu8 " reset complete, pump is now available", pump_idx);
  // Post reset event to get the controller out of fault state
  if (this->hsm_.is_ready()) {
    this->hsm_.post_event(EVT_PUMP_RESET);
  }
}

void SprinklifyController::on_manual_pump_run_requested(uint8_t pump_idx, bool run) {
  if (!this->hsm_.is_ready())
    return;

  if (run) {
    // Reject if another pump is already active
    if (this->active_pump_idx_ != NO_PUMP) {
      ESP_LOGW(TAG, "Manual start pump %" PRIu8 ": pump %" PRIu8 " already active — ignoring", pump_idx,
               this->active_pump_idx_);
#ifdef USE_SWITCH
      if (this->pumps_[pump_idx].run_switch != nullptr)
        this->pumps_[pump_idx].run_switch->sync_state(false);
#endif
      return;
    }
    ESP_LOGI(TAG, "Manual run requested for pump %" PRIu8, pump_idx);
    this->requested_pump_idx_ = pump_idx;
    this->hsm_.post_event(EVT_MANUAL_START_REQUESTED);
  } else {
    // Only act if this is the pump that's currently running
    if (this->active_pump_idx_ != pump_idx) {
      // Harmless: switch turned off for a pump that wasn't running anyway
      return;
    }
    ESP_LOGI(TAG, "Manual stop requested for pump %" PRIu8, pump_idx);
    this->hsm_.post_event(EVT_MANUAL_STOP_REQUESTED);
  }
}

void SprinklifyController::on_master_trigger_changed(bool active) {
  if (!this->hsm_.is_ready())
    return;

  // Manual mode: master trigger is disabled — only HA controls work.
  if (!this->is_auto_mode_()) {
    ESP_LOGD(TAG, "Master trigger %s — ignored (manual mode)", active ? "active" : "inactive");
    return;
  }

  if (active) {
    if (this->active_pump_idx_ != NO_PUMP) {
      ESP_LOGD(TAG, "Master trigger active but pump %" PRIu8 " already running — ignoring", this->active_pump_idx_);
      return;
    }
    const uint8_t idx = this->get_first_available_pump_();
    if (idx == NO_PUMP) {
      ESP_LOGW(TAG, "Master trigger active but no pumps available");
      return;
    }
    ESP_LOGI(TAG, "Master trigger: starting pump %" PRIu8, idx);
    this->requested_pump_idx_ = idx;
    this->hsm_.post_event(EVT_MASTER_TRIGGER_ACTIVE);
  } else {
    ESP_LOGI(TAG, "Master trigger inactive: requesting stop");
    this->hsm_.post_event(EVT_MASTER_TRIGGER_INACTIVE);
  }
}

void SprinklifyController::led_tick_callback_() {
  this->pressure_led_.tick();
  this->flow_led_.tick();
  this->status_led_green_.tick();
  this->status_led_red_.tick();
  for (uint8_t i = 0; i < PUMP_COUNT; i++) {
    this->pumps_[i].led.tick();
  }
}

// ---------------------------------------------------------------------------
// Other helpers
// ---------------------------------------------------------------------------

/// @brief Finds the first available pump in the system.
/// @return Index of pump, or NO_PUMP (-1) if no available pump found.
uint8_t SprinklifyController::get_first_available_pump_() const {
  for (size_t i = 0; i < PUMP_COUNT; ++i) {
    if (this->pumps_[i].is_available())
      return i;
  }
  return NO_PUMP;
}

/// @brief Starts the next pump, as specified in the next_pump_idx_ member variable.
bool SprinklifyController::start_next_pump_() {
  const uint8_t next_idx = this->get_first_available_pump_();
  if (next_idx == NO_PUMP) {
    ESP_LOGW(TAG, "No pumps available to start!");
    return false;
  }
  ESP_LOGD(TAG, "Starting pump %" PRIu8, next_idx);
  this->active_pump_idx_ = next_idx;
  auto *pump = &this->pumps_[this->active_pump_idx_];
  pump->turn_on();
  pump->set_status(PumpStatus::PUMP_STARTING);
  pump->start_run();

  // Reset flow sensor's total counter
  this->flow_sensor_->reset_total();
  return true;
}

void SprinklifyController::stop_active_pump_(bool faulted, bool latched) {
  if (this->active_pump_idx_ == NO_PUMP) {
    ESP_LOGW(TAG, "Attempted to stop pump but no active pump index set");
    return;
  }
  ESP_LOGD(TAG, "Stopping pump %" PRIu8, this->active_pump_idx_);
  float vol = this->flow_sensor_->get_total_liters();
  auto *pump = &this->pumps_[this->active_pump_idx_];
  pump->turn_off();
  pump->stop_run(vol);
  if (faulted) {
    pump->record_fault(latched, this->now_);
    pump->set_status(PumpStatus::PUMP_FAULT);
    // Arm auto-reset for this pump
    uint8_t i = this->active_pump_idx_;
    const time_t reset_at = this->now_ + this->pumps_[i].get_auto_reset_wait_time_ms() / 1000;
    pump->auto_reset_event_id = this->scheduler_.add_onetime_event(reset_at, [this, i]() { this->on_reset_pump(i); });
  } else {
    pump->set_status(PumpStatus::PUMP_AVAILABLE);
  }
  pump->save();
  // Post event to HSM to trigger interlock and next steps
  this->hsm_.post_event(EVT_PUMP_STOPPED);
  // Push run switch back to OFF so HA reflects the stopped state - only relevant in manual mode
#ifdef USE_SWITCH
  if (this->pumps_[this->active_pump_idx_].run_switch != nullptr)
    this->pumps_[this->active_pump_idx_].run_switch->sync_state(false);
#endif
  this->active_pump_idx_ = NO_PUMP;
}

/// @brief Starts a specific pump by index (used by manual mode).
/// Unlike start_next_pump_(), this respects the caller's pump choice but
/// still enforces availability (not faulted, installed, relay wired).
/// @return true if the pump was started, false if it was unavailable.
bool SprinklifyController::start_specific_pump_(uint8_t idx) {
  if (idx >= PUMP_COUNT) {
    ESP_LOGW(TAG, "start_specific_pump_: index %" PRIu8 " out of range", idx);
    return false;
  }
  if (!this->pumps_[idx].is_available()) {
    ESP_LOGW(TAG, "start_specific_pump_: pump %" PRIu8 " not available (faulted or not installed)", idx);
    return false;
  }
  ESP_LOGI(TAG, "Manual: starting pump %" PRIu8, idx);
  this->active_pump_idx_ = idx;
  auto *pump = &this->pumps_[idx];
  pump->turn_on();
  pump->set_status(PumpStatus::PUMP_STARTING);
  pump->start_run();
  this->flow_sensor_->reset_total();
#ifdef USE_SWITCH
  if (pump->run_switch != nullptr) {
    pump->run_switch->sync_state(true);
  }
#endif
  return true;
}

void SprinklifyController::reset_pump_switches_() {
#ifdef USE_SWITCH
  for (uint8_t i = 0; i < PUMP_COUNT; i++) {
    if (this->pumps_[i].run_switch != nullptr)
      this->pumps_[i].run_switch->sync_state(false);
  }
#endif
}

void SprinklifyController::publish_controller_state_() {
  uint16_t red_pattern;
  uint16_t green_pattern;

  switch (this->hsm_.current_state()) {
    case ControllerStates::AUTO_IDLE:
      red_pattern = SprinklifyLEDIndicator::PATTERN_OFF;
      green_pattern = SprinklifyLEDIndicator::PATTERN_BLINK_SLOW;
      break;
    case ControllerStates::AUTO_PUMPING_WAITING_FOR_FLOW:
      red_pattern = SprinklifyLEDIndicator::PATTERN_OFF;
      green_pattern = SprinklifyLEDIndicator::PATTERN_BLINK_FAST;
      break;
    case ControllerStates::AUTO_PUMPING_RUNNING:
      red_pattern = SprinklifyLEDIndicator::PATTERN_OFF;
      green_pattern = SprinklifyLEDIndicator::PATTERN_SOLID_ON;
      break;
    case ControllerStates::AUTO_FAULT:
      red_pattern = SprinklifyLEDIndicator::PATTERN_BLINK_SLOW;
      green_pattern = SprinklifyLEDIndicator::PATTERN_OFF;
      break;
    case ControllerStates::MANUAL_IDLE:
      red_pattern = SprinklifyLEDIndicator::PATTERN_OFF;
      green_pattern = SprinklifyLEDIndicator::PATTERN_PULSE;
      break;
    case ControllerStates::MANUAL_RUNNING:
      red_pattern = SprinklifyLEDIndicator::PATTERN_OFF;
      green_pattern = SprinklifyLEDIndicator::PATTERN_DOUBLE;
      break;
    case ControllerStates::INTERLOCK_WAIT:
      red_pattern = SprinklifyLEDIndicator::PATTERN_DOUBLE;
      green_pattern = SprinklifyLEDIndicator::PATTERN_DOUBLE;
      break;
    default:
      red_pattern = SprinklifyLEDIndicator::PATTERN_SOLID_ON;
      green_pattern = SprinklifyLEDIndicator::PATTERN_SOLID_ON;
      break;
  }
  this->status_led_red_.set_pattern(red_pattern);
  this->status_led_green_.set_pattern(green_pattern);

  ESP_LOGD(TAG, "New controller state: %s", this->state_as_str_(this->hsm_.current_state()));

#ifdef USE_TEXT_SENSOR
  if (this->controller_state_sensor_ != nullptr) {
    this->controller_state_sensor_->publish_state(this->state_as_str_(this->hsm_.current_state()));
  }
#endif
}

#ifdef USE_BINARY_SENSOR
void SprinklifyController::update_binary_sensor_(BinarySensorType type, bool value) {
  if (type < BinarySensorType::BINARY_SENSOR_TYPE_COUNT) {
    size_t index = (size_t) type;
    if ((this->binary_sensors_[index] != nullptr) &&
        ((!this->binary_sensors_[index]->has_state()) || (this->binary_sensors_[index]->state != value)))
      this->binary_sensors_[index]->publish_state(value);
  }
}
#endif

}  // namespace sprinklify
}  // namespace esphome
