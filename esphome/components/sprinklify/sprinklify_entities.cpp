#include "sprinklify_entities.h"
#include "sprinklify_controller.h"  // full definition needed here, not in .h

namespace esphome {
namespace sprinklify {

// --- PumpResetButton ---

#ifdef USE_BUTTON
void PumpResetButton::press_action() { this->parent_->on_reset_pump(pump_index_); }
#endif

}  // namespace sprinklify
}  // namespace esphome
