#include "winter_mode.h"

namespace esphome {
namespace sprinklify {

static const char *const TAG = "winter_mode.switch";

void WinterModeSwitch::setup() {
  auto initial = this->get_initial_state_with_restore_mode();
  if (initial.has_value()) {
    this->write_state(*initial);
  }
}

void WinterModeSwitch::dump_config() { LOG_SWITCH("  ", "WinterMode Switch", this); }

void WinterModeSwitch::write_state(bool state) {
  this->publish_state(state);
  this->parent_->on_winter_mode_changed(state);
}

}  // namespace sprinklify
}  // namespace esphome
