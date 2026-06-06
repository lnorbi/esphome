#include "sprinklify_entities.h"
#include "sprinklify_controller.h"  // full definition needed here, not in .h

namespace esphome {
namespace sprinklify {

static const char *const TAG = "sprinklify.entities";

// --- PumpResetButton ---

#ifdef USE_BUTTON
void PumpResetButton::press_action() { this->parent_->on_reset_pump(pump_index_); }
#endif

#ifdef USE_SWITCH
// --- PumpInstalledSwitch ---
void PumpInstalledSwitch::setup() {
  auto initial = this->get_initial_state_with_restore_mode();
  if (initial.has_value()) {
    this->write_state(*initial);
  }
}

void PumpInstalledSwitch::dump_config() {
  LOG_SWITCH("  ", "Pump Installed Switch", this);
  ESP_LOGCONFIG(TAG, "    Pump index: %" PRIu8, this->pump_index_);
}

void PumpInstalledSwitch::write_state(bool state) {
  this->publish_state(state);
  this->parent_->on_pump_installed_changed(this->pump_index_, state);
}

// --- PumpRunSwitch ---
void PumpRunSwitch::dump_config() {
  LOG_SWITCH("  ", "Pump Run Switch", this);
  ESP_LOGCONFIG(TAG, "    Pump index: %" PRIu8, this->pump_index_);
}

void PumpRunSwitch::write_state(bool state) {
  this->publish_state(state);
  if (!this->hub_sync_) {
    this->parent_->on_manual_pump_run_requested(pump_index_, state);
  }
}
#endif

}  // namespace sprinklify
}  // namespace esphome
