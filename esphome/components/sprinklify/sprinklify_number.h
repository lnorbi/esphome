#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/number/number.h"

namespace esphome {
namespace sprinklify {

// Forward declaration — SprinklifyNumber is Parented to SprinklifyController,
// but does not call any hub methods. The parent pointer is kept for future
// extensibility and to satisfy the Parented<> convention used by all
// Sprinklify child entities.
class SprinklifyController;

/// @brief A self-contained number entity that persists its value to NVS
/// using make_entity_preference<float>() — the same mechanism used by
/// ESPHome built-in number platforms.
///
/// Persistence is fully owned by this class. The hub is not involved
/// in save or restore. The hub's accessor functions read .state directly,
/// falling back to the compile-time default if needed.
///
/// Lifecycle:
///   setup()   — restores from NVS, or falls back to initial_value_,
///               or falls back to min_value. Publishes the resolved value.
///   control() — called by HA / frontend when the user sets a new value.
///               Publishes and saves to NVS immediately.
class SprinklifyNumber : public Component, public number::Number, public Parented<SprinklifyController> {
 public:
  /// @brief Sets the initial value used when no persisted value exists.
  /// Should be set from codegen to the YAML compile-time default,
  /// so that first-boot behaviour matches the user's stated intent.
  void set_initial_value(float v) { initial_value_ = v; }

  // --- ESPHome lifecycle ---

  /// @brief Restores persisted value or initialises from initial_value_ /
  /// min_value. Always results in a valid (non-NaN) published state before
  /// the hub's setup() reads .state via its accessors.
  void setup() override;

  void dump_config() override;

  /// @brief Numbers should initialise after the hub (DATA priority) so that
  /// the hub's setup() can safely read restored values via accessors.
  float get_setup_priority() const override { return setup_priority::DATA - 1.0f; }

 protected:
  /// @brief Called by the ESPHome number infrastructure when the value is
  /// changed from HA or the web UI. Publishes the new state and saves to NVS.
  void control(float value) override;

 private:
  ESPPreferenceObject pref_;
  float initial_value_{NAN};
};

}  // namespace sprinklify
}  // namespace esphome
