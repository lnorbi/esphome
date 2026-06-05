#include "sprinklify_entities.h"
#include "sprinklify_controller.h"  // full definition needed here, not in .h

namespace esphome {
namespace sprinklify {

// --- PumpResetButton ---

#ifdef USE_BUTTON
void PumpResetButton::press_action() { this->parent_->on_reset_pump(pump_index_); }
#endif

#ifdef USE_SWITCH
void PumpRunSwitch::write_state(bool state) {
  this->publish_state(state);
  if (!this->hub_sync_) {
    this->parent_->on_manual_pump_run_requested(pump_index_, state);
  }
}
#endif

}  // namespace sprinklify
}  // namespace esphome
