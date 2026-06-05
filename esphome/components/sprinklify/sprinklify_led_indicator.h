#pragma once

#include "esphome/components/output/binary_output.h"

namespace esphome {
namespace sprinklify {

/// @brief A lightweight LED driver that produces any on/off pattern by
/// replaying a 16-bit bitmask at a fixed tick rate.
///
/// Each bit represents one tick interval (default 100 ms), giving a full
/// cycle of 1.6 seconds. Bit 0 is the first bit played; the cursor advances
/// circularly through all 16 bits.
///
/// Usage:
///   1. Call set_output() once from the hub's wiring phase.
///   2. Call set_pattern() whenever the LED behaviour should change
///      (typically from publish_pump_status_() or a state EVT_ENTRY).
///   3. Call tick() every 100 ms from the hub's loop().
///
/// The class is header-only — no .cpp required.
struct SprinklifyLEDIndicator {
 public:
  // ---------------------------------------------------------------------------
  // Pre-defined patterns
  //
  // Each pattern is a 16-bit bitmask. Bit position N is the output state
  // during tick N of the 1.6 s cycle. Read right-to-left: bit 0 is first.
  //
  // Examples at 100 ms / tick:
  //   PATTERN_ON          solid on
  //   PATTERN_OFF         solid off
  //   PATTERN_SLOW_FLASH  on 800 ms, off 800 ms  (~0.6 Hz)
  //   PATTERN_FAST_FLASH  on 100 ms, off 100 ms  (~5 Hz)
  //   PATTERN_PULSE       brief 100 ms pulse once per 1.6 s cycle
  //   PATTERN_DOUBLE      two brief pulses per 1.6 s cycle
  //   PATTERN_TRIPLE      three brief pulses per 1.6 s cycle
  // ---------------------------------------------------------------------------
  static constexpr uint16_t PATTERN_OFF = 0b0000000000000000;
  static constexpr uint16_t PATTERN_SOLID_ON = 0b1111111111111111;
  static constexpr uint16_t PATTERN_BLINK_SLOW = 0b0000000011111111;
  static constexpr uint16_t PATTERN_BLINK_FAST = 0b0101010101010101;
  static constexpr uint16_t PATTERN_PULSE = 0b0000000000000001;
  static constexpr uint16_t PATTERN_DOUBLE = 0b0000000100000001;
  static constexpr uint16_t PATTERN_TRIPLE = 0b0000000100010001;

  // ---------------------------------------------------------------------------
  // Wiring
  // ---------------------------------------------------------------------------

  /// @brief Sets the GPIO output this indicator drives.
  /// Must be called before the first tick(). Safe to call with nullptr —
  /// tick() is a no-op when no output is set.
  void set_output(output::BinaryOutput *out) { this->output_ = out; }

  // ---------------------------------------------------------------------------
  // Pattern control
  // ---------------------------------------------------------------------------

  /// @brief Sets the active bitmask pattern and resets the bit cursor to 0.
  /// Resetting the cursor ensures the new pattern always starts from bit 0,
  /// giving a clean, predictable transition regardless of when in the cycle
  /// the change occurs.
  void set_pattern(uint16_t pattern) {
    if (this->pattern_ == pattern) {
      return;
    }
    this->pattern_ = pattern;
    this->bit_ = 0;
  }

  /// @brief Returns the currently active pattern.
  uint16_t get_pattern() const { return this->pattern_; }

  /// @brief Returns true if the output is currently driven high.
  bool is_on() const { return (this->pattern_ >> this->bit_) & 1U; }

  // ---------------------------------------------------------------------------
  // Tick — call every 100 ms from the hub's loop()
  // ---------------------------------------------------------------------------

  /// @brief Advances the pattern by one bit and updates the output.
  /// Must be called at a fixed interval (100 ms recommended) for patterns
  /// to produce the correct timing. Safe to call when output_ is nullptr.
  void tick() {
    if (this->output_ == nullptr) {
      return;
    }
    const bool state = (this->pattern_ >> this->bit_) & 1U;
    if (state) {
      this->output_->turn_on();
    } else {
      this->output_->turn_off();
    }

    // this->output_->set_state(true);
    this->bit_ = (this->bit_ + 1) & 15U;  // circular 0..15
  }

  /// @brief Forces the output to a known off state immediately, without
  /// advancing the pattern cursor. Useful for safe shutdown or OTA.
  void force_off() {
    if (this->output_ == nullptr) {
      return;
    }
    this->output_->set_state(false);
  }

 private:
  output::BinaryOutput *output_{nullptr};
  uint16_t pattern_{PATTERN_OFF};
  uint8_t bit_{0};
};

}  // namespace sprinklify
}  // namespace esphome
