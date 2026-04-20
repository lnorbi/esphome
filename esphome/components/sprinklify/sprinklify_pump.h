#pragma once

#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "esphome/components/output/binary_output.h"
#include "esphome/components/sensor/sensor.h"

#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif

#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif

#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#include "sprinklify_led_indicator.h"
#include "scheduled_event.h"
#include <cmath>

namespace esphome {
namespace sprinklify {

enum class PumpStatus : uint8_t {
  PUMP_AVAILABLE,    // idle, no fault, can be requested
  PUMP_STARTING,     // energised, waiting for flow to confirm
  PUMP_RUNNING,      // flow confirmed, operating normally
  PUMP_FAULT,        // latched fault — requires reset before next run
  PUMP_UNAVAILABLE,  // reserved for future use (e.g. maintenance mode)
};

constexpr float MINUTES_TO_MILLISECONDS = 60.0f * 1000.0f;
constexpr float HOURS_TO_MILLISECONDS = 60.0f * MINUTES_TO_MILLISECONDS;

// ---------------------------------------------------------------------------
// Persistent pump data — serialised to NVS via ESPPreferenceObject.
//
// IMPORTANT: changing the layout of this struct (adding, removing, or
// reordering fields) MUST be accompanied by bumping CURRENT_MAGIC.
// A magic mismatch causes the hub to initialise fresh data rather than
// loading stale bytes, preventing silent corruption.
// ---------------------------------------------------------------------------
struct PumpPersistentData {
  uint32_t magic{0};  // version/sanity check — invalidates stale data on schema change

  // Fault state — fault_latched survives reboots intentionally:
  // a pump that faulted before a power cut must still be faulted on return.
  bool fault_latched{false};  // true = timeout fault, manual ack required
  time_t last_fault_unix{0};  // wall-clock timestamp of last fault (0 = unknown)

  // Cumulative statistics — published to HA on pump stop, not continuously
  uint32_t total_runtime_s{0};  // lifetime running time in seconds
  float total_volume_l{0.0f};   // lifetime volume pumped in litres

  static constexpr uint32_t CURRENT_MAGIC = 0x50435635;  // "PCV5"
};

// ---------------------------------------------------------------------------
// Pump — self-contained per-pump data, config, entities, and behaviour.
//
// Compile-time config (relay, timing) is set once by codegen via add_pump()
// and index-based setters. Runtime mutable config arrives via optional Number
// entities, read through accessors.
//
// NVS persistence is fully owned here: init_pref(), load(), save().
// The controller calls these; no NVS logic lives in sprinklify_controller.cpp.
//
// installed state is owned by the optional InstalledSwitch entity using
// ESPHome's standard restore_mode mechanism. is_installed() returns true
// when no switch is wired, ensuring pumps are active by default.
// ---------------------------------------------------------------------------
struct Pump {
  // =========================================================================
  // Hardware
  // =========================================================================

  output::BinaryOutput *relay{nullptr};
  SprinklifyLEDIndicator led;

  // =========================================================================
  // Compile-time configuration — set by codegen, immutable after setup()
  // =========================================================================

  uint32_t max_runtime_ms{1800000};     // 30 min default
  uint32_t auto_reset_wait_time_ms{0};  // 0 = disabled

  // =========================================================================
  // Optional child entities
  // =========================================================================
  sensor::Sensor *total_volume_sensor{nullptr};
  sensor::Sensor *total_runtime_sensor{nullptr};

#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *state_sensor{nullptr};
#endif

#ifdef USE_SWITCH
  switch_::Switch *installed_switch{nullptr};
#endif

#ifdef USE_NUMBER
  number::Number *max_runtime_number{nullptr};
  number::Number *auto_reset_wait_time_number{nullptr};
#endif

  // =========================================================================
  // NVS preference handle
  // =========================================================================

  ESPPreferenceObject pref;
  PumpPersistentData persistent{};

  // =========================================================================
  // Volatile runtime state — not persisted
  // =========================================================================

  PumpStatus status{PumpStatus::PUMP_AVAILABLE};
  uint32_t run_start_ms{0};                        // millis() at energise; 0 = not running
  uint16_t auto_reset_event_id{INVALID_EVENT_ID};  // ID of pending auto-reset event, or INVALID_EVENT_ID if none
  char unique_name_[24] = "UNKNOWN";

  // =========================================================================
  // Configuration accessors — resolve Number override or fall back to
  // compile-time value. Always use these; never read the _ms fields directly.
  // =========================================================================
  uint32_t get_max_runtime_ms() const {
#ifdef USE_NUMBER
    if (this->max_runtime_number != nullptr && this->max_runtime_number->has_state())
      return static_cast<uint32_t>(this->max_runtime_number->state * MINUTES_TO_MILLISECONDS);  // Mins to ms
#endif
    return this->max_runtime_ms;
  }

  uint32_t get_auto_reset_wait_time_ms() const {
#ifdef USE_NUMBER
    if (this->auto_reset_wait_time_number != nullptr && this->auto_reset_wait_time_number->has_state())
      return static_cast<uint32_t>(this->auto_reset_wait_time_number->state * HOURS_TO_MILLISECONDS);  // Hours to ms
#endif
    return this->auto_reset_wait_time_ms;
  }

  // =========================================================================
  // State queries
  // =========================================================================

  bool is_installed() const {
#ifdef USE_SWITCH
    return this->installed_switch == nullptr || this->installed_switch->state;
#endif
    return true;
  }
  bool is_available() const { return this->is_installed() && this->status == PumpStatus::PUMP_AVAILABLE; }
  bool is_faulted() const { return this->status == PumpStatus::PUMP_FAULT; }

  // =========================================================================
  // Status — single setter keeps state sensor and LED always in sync
  // =========================================================================

  /// @brief Sets pump status, publishes to state_sensor, and updates LED.
  /// Always use this instead of assigning status directly.
  void set_status(PumpStatus new_status) {
    this->status = new_status;
    this->led.set_pattern(this->led_pattern_for_status_(new_status));

#ifdef USE_TEXT_SENSOR
    if (this->state_sensor != nullptr) {
      this->state_sensor->publish_state(this->pump_status_to_str_(new_status));
    }
#endif
  }

  // =========================================================================
  // Hardware control
  // =========================================================================

  void turn_on() { this->relay->turn_on(); }
  void turn_off() { this->relay->turn_off(); }

  // =========================================================================
  // Run timing
  // =========================================================================

  /// @brief Records the run start timestamp. Call when a pump cycle is started.
  void start_run() { this->run_start_ms = millis(); }

  /// @brief Accumulates elapsed time into persistent.total_runtime_s
  /// and resets the run timer. Returns elapsed seconds.
  /// Call before save() on every pump stop.
  uint32_t stop_run(float volume_delta_l) {
    if (this->run_start_ms == 0) {
      return 0;
    }
    const uint32_t elapsed_s = (millis() - this->run_start_ms) / 1000;  // seconds
    this->persistent.total_runtime_s += elapsed_s;
    this->persistent.total_volume_l += volume_delta_l;
    this->run_start_ms = 0;
    // Update sensors
    if (this->total_runtime_sensor != nullptr) {
      this->total_runtime_sensor->publish_state(this->persistent.total_runtime_s / 60.0f);  // minutes
    }
    if (this->total_volume_sensor != nullptr) {
      this->total_volume_sensor->publish_state(this->persistent.total_volume_l);
    }
    return elapsed_s;
  }

  /// @brief Publishes last known statistic values to HA on boot.
  /// Call from the controller's setup() after load() and fault resolution,
  /// so HA sees current values immediately without waiting for the next run.
  void restore_sensors() {
    if (this->total_runtime_sensor != nullptr) {
      this->total_runtime_sensor->publish_state(this->persistent.total_runtime_s / 60.0f);
    }
    if (this->total_volume_sensor != nullptr) {
      this->total_volume_sensor->publish_state(this->persistent.total_volume_l);
    }
    // State sensor restored separately by the hub after fault logic runs
  }

  // =========================================================================
  // Fault management
  // =========================================================================

  /// @brief Records a fault into persistent data.
  /// latched = true for timeout faults (manual ack required).
  /// latched = false for dry-run faults (auto-reset eligible).
  /// Caller must call save() after this.
  void record_fault(bool latched, time_t fault_unix) {
    this->persistent.fault_latched = latched;
    this->persistent.last_fault_unix = fault_unix;
  }

  void clear_fault() {
    this->persistent.fault_latched = false;
    this->persistent.last_fault_unix = 0;
  }

  /// @brief Clears fault state, persists to NVS, and sets status to AVAILABLE.
  /// Does not cancel any pending scheduler events — the controller handles that.
  void reset() {
    this->clear_fault();
    this->auto_reset_event_id = INVALID_EVENT_ID;  // invalidate any pending auto-reset event
    this->save();
    this->set_status(PumpStatus::PUMP_AVAILABLE);
  }

  // =========================================================================
  // NVS persistence
  //
  // Ownership: the Pump struct owns its NVS slot.
  // The controller calls init_pref(index) once in setup(), then
  // load() to restore, save() after any state change worth persisting.
  // =========================================================================

  /// @brief Binds this pump's NVS slot using a stable hash string.
  /// MUST be called before load() or save(). The hash string is a permanent
  /// contract — changing it across firmware updates silently loses all data.
  void init_pref(uint8_t pump_index) {
    snprintf(this->unique_name_, sizeof(this->unique_name_), "sprinklify_pump_%u", pump_index);
    this->pref = global_preferences->make_preference<PumpPersistentData>(fnv1a_hash(this->unique_name_));
  }

  /// @brief Loads persistent data from NVS. Returns true if valid data was
  /// found with the correct magic. On false, caller should reset to defaults.
  bool load() {
    if (!this->pref.load(&this->persistent) || this->persistent.magic != PumpPersistentData::CURRENT_MAGIC) {
      // No valid data — initialise fresh defaults and save to NVS to claim the slot
      this->persistent = PumpPersistentData{};
      this->persistent.magic = PumpPersistentData::CURRENT_MAGIC;
      this->save();
      return false;
    }
    return true;
  }

  /// @brief Saves current persistent data to NVS.
  void save() { this->pref.save(&this->persistent); }

 private:
#ifdef USE_TEXT_SENSOR
  static const char *pump_status_to_str_(PumpStatus s) {
    switch (s) {
      case PumpStatus::PUMP_AVAILABLE:
        return "available";
      case PumpStatus::PUMP_STARTING:
        return "starting";
      case PumpStatus::PUMP_RUNNING:
        return "running";
      case PumpStatus::PUMP_FAULT:
        return "fault";
      case PumpStatus::PUMP_UNAVAILABLE:
        return "unavailable";
      default:
        return "unknown";
    }
  }
#endif

  static uint16_t led_pattern_for_status_(PumpStatus s) {
    switch (s) {
      case PumpStatus::PUMP_AVAILABLE:
        return SprinklifyLEDIndicator::PATTERN_OFF;
      case PumpStatus::PUMP_STARTING:
        return SprinklifyLEDIndicator::PATTERN_BLINK_FAST;
      case PumpStatus::PUMP_RUNNING:
        return SprinklifyLEDIndicator::PATTERN_SOLID_ON;
      case PumpStatus::PUMP_FAULT:
        return SprinklifyLEDIndicator::PATTERN_PULSE;
      default:
        return SprinklifyLEDIndicator::PATTERN_OFF;
    }
  }
};

}  // namespace sprinklify
}  // namespace esphome
