#pragma once

#include "esphome/core/helpers.h"
#include "esphome/core/time.h"
#include <functional>

namespace esphome {
namespace sprinklify {

enum class ScheduledEventType { ONE_TIME, DAILY };

/// @brief Sentinel returned by EventScheduler::add_*() on failure.
static constexpr uint16_t INVALID_EVENT_ID = 0xFFFF;

struct ScheduledEvent {
  using CallbackFunction = std::function<void()>;

  uint16_t id{INVALID_EVENT_ID};  // Scheduler-assigned; stable across other removals

  CallbackFunction callback;
  ScheduledEventType type;

  uint8_t second{0};
  uint8_t minute{0};
  uint8_t hour{0};
  time_t timestamp{0};

  bool should_remove{false};
  bool last_check_valid{false};
  ESPTime last_check;

  // One-time event
  // ScheduledEvent(uint16_t id, time_t evt_at, CallbackFunction cb)
  //     : id(id), callback(std::move(cb)), type(ScheduledEventType::ONE_TIME), timestamp(evt_at) {}

  // Daily event
  // ScheduledEvent(uint16_t id, uint8_t h, uint8_t m, uint8_t s, CallbackFunction cb)
  //     : id(id), callback(std::move(cb)), type(ScheduledEventType::DAILY), hour(h), minute(m), second(s) {}

  // One-time event
  void set_one_time(uint16_t id, time_t evt_at, CallbackFunction cb) {
    this->id = id;
    this->type = ScheduledEventType::ONE_TIME;
    this->timestamp = evt_at;
    this->callback = std::move(cb);
  }

  // Daily event
  void set_daily(uint16_t id, uint8_t h, uint8_t m, uint8_t s, CallbackFunction cb) {
    this->id = id;
    this->type = ScheduledEventType::DAILY;
    this->hour = h;
    this->minute = m;
    this->second = s;
    this->callback = std::move(cb);
  }

  /// @brief Checks if the repeating event's time matches the time argument given.
  /// @param time ESPTime time value to compare to.
  /// @return True if the event matches the time given, false otherwise.
  inline bool matches(const ESPTime &time) const {
    if (this->type == ScheduledEventType::ONE_TIME)
      return false;
    return this->second == time.second && this->minute == time.minute && this->hour == time.hour;
  }

  /// @brief Processes a scheduled event, checking whether it should fire.
  /// @param time The current time.
  void process(const ESPTime &time) {
    if (this->should_remove)
      return;
    // Need valid time
    if (!time.is_valid())
      return;
    if (this->type == ScheduledEventType::ONE_TIME) {
      if (time.timestamp >= this->timestamp) {
        this->callback();
        this->should_remove = true;
      }
    } else {
      if (this->last_check_valid) {
        while (true) {
          this->last_check.increment_second();
          if (this->last_check >= time)
            break;

          if (this->matches(this->last_check)) {
            this->callback();
          }
        }
      }

      this->last_check = time;
      this->last_check_valid = true;
      if (this->matches(time)) {
        this->callback();
      }
    }
  }
};

}  // namespace sprinklify
}  // namespace esphome
