#include "auto_mode.h"

namespace esphome {
namespace sprinklify {

void AutoModeSwitch::setup() {
  auto initial = this->get_initial_state_with_restore_mode();
  if (initial.has_value()) {
    this->write_state(*initial);
  }
}

void AutoModeSwitch::write_state(bool state) {
  // if (this->parent_->get_auto_mode() != state) {
  // }
  this->parent_->on_auto_mode_changed(state);
  this->publish_state(state);
}

}  // namespace sprinklify
}  // namespace esphome
