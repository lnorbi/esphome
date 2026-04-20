#include "sprinklify_number.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sprinklify {

static const char *const TAG = "sprinklify.number";

void SprinklifyNumber::setup() {
  float value;

  this->pref_ = this->make_entity_preference<float>();

  if (this->pref_.load(&value)) {
    // Persisted value found — use it.
    ESP_LOGD(TAG, "'%s': Restored value %.3f", this->get_name().c_str(), value);
  } else if (!std::isnan(this->initial_value_)) {
    // No persisted value — fall back to the YAML compile-time default.
    value = this->initial_value_;
    ESP_LOGD(TAG, "'%s': No saved value — using initial value %.3f", this->get_name().c_str(), value);
  } else {
    // No persisted value and no initial value — use min_value as a safe floor.
    value = this->traits.get_min_value();
    ESP_LOGW(TAG, "'%s': No saved or initial value — falling back to min %.3f", this->get_name().c_str(), value);
  }

  this->publish_state(value);
}

void SprinklifyNumber::dump_config() {
  LOG_NUMBER("", "Sprinklify Number", this);
  ESP_LOGCONFIG(TAG, "  Initial value: %.3f", this->initial_value_);
}

void SprinklifyNumber::control(float value) {
  this->publish_state(value);
  this->pref_.save(&value);
  ESP_LOGD(TAG, "'%s': Saved value %.3f", this->get_name().c_str(), value);
}

}  // namespace sprinklify
}  // namespace esphome
