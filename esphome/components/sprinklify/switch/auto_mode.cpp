#include "auto_mode.h"

namespace esphome {
namespace sprinklify {

static const char *const TAG = "auto_mode.switch";

void AutoModeSwitch::setup() {
  auto initial = this->get_initial_state_with_restore_mode();
  if (initial.has_value()) {
    this->write_state(*initial);
  }
}

void AutoModeSwitch::dump_config() { LOG_SWITCH("  ", "AutoMode Switch", this); }

void AutoModeSwitch::write_state(bool state) {
  this->publish_state(state);
  this->parent_->on_auto_mode_changed(state);
}

}  // namespace sprinklify
}  // namespace esphome
