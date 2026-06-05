#pragma once

#include "esphome/core/component.h"

#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif

#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif

// SprinklifyPressureSensor is now a dedicated Component with its own files —
// see sprinklify_pressure_sensor.h. It is no longer declared here.

namespace esphome {
namespace sprinklify {

class SprinklifyController;

// ---------------------------------------------------------------------------
// Text sensor entities — read-only, published by the hub on state transitions.
// ---------------------------------------------------------------------------

#ifdef USE_BUTTON
// ---------------------------------------------------------------------------
// Button entities — write-only from HA, each delegates to the hub.
// ---------------------------------------------------------------------------

/// @brief Resets the latched fault state for a specific pump.
/// The pump index is set by codegen via set_pump_index(), mirroring the
/// pattern used by all per-pump child entities.
/// Posts EVT_FAULT_ACK to the HSM after setting requested_pump_index_
/// on the hub.
class PumpResetButton : public button::Button, public Parented<SprinklifyController> {
 public:
  void set_pump_index(uint8_t idx) { pump_index_ = idx; }

 protected:
  void press_action() override;
  uint8_t pump_index_{0};
};
#endif

#ifdef USE_SWITCH
// ---------------------------------------------------------------------------
// Switch entities
// ---------------------------------------------------------------------------

/// @brief Per-pump installed/enabled switch.
/// Disabling a pump (installed = false) prevents it from being selected
/// by the pump selection algorithm without requiring a firmware recompile.
///
/// State is persisted in PumpPersistentData (NVS) — hub owns the persistence.
/// DISABLED restore_mode prevents the switch restoring itself on boot;
/// hub calls publish_state(persistent.installed) explicitly in setup().
class InstalledSwitch : public switch_::Switch, public Parented<SprinklifyController> {
 public:
  void set_pump_index(uint8_t idx) { pump_index_ = idx; }

 protected:
  void write_state(bool state) override {
    this->publish_state(state);
    // this->parent_->on_pump_installed_changed(pump_index_, state);
  }
  uint8_t pump_index_{0};
};

/// @brief Per-pump manual run switch.
/// When turned on from HA, requests the hub to start this specific pump in
/// manual mode. When turned off, requests the hub to stop it.
///
/// The hub is authoritative over the actual state — it calls publish_state()
/// back to HA once the pump actually starts/stops, so the switch reflects
/// reality, not just the request.
///
/// Behaviour in auto mode: the hub ignores the event silently.
/// Behaviour when the pump is faulted: the hub rejects the start and
/// the switch is pushed back to OFF via publish_state(false).
class PumpRunSwitch : public switch_::Switch, public Parented<SprinklifyController> {
 public:
  void set_pump_index(uint8_t idx) { pump_index_ = idx; }
  // Called by the hub to sync HA state without triggering a callback loop.
  void sync_state(bool state) {
    this->hub_sync_ = true;
    this->publish_state(state);
    this->hub_sync_ = false;
  }

 protected:
  void write_state(bool state) override;

  uint8_t pump_index_{0};
  bool hub_sync_{false};
};

#endif

// ---------------------------------------------------------------------------
// Sensor entities — read-only from HA, published by the hub on state transitions.
// ---------------------------------------------------------------------------

}  // namespace sprinklify
}  // namespace esphome
