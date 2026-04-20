// event_scheduler.h
#pragma once

#include "esphome/core/helpers.h"
#include "scheduled_event.h"

#include <array>

namespace esphome {
namespace sprinklify {

static constexpr size_t MAX_EVENTS = 8;

class EventScheduler {
 public:
  // ── Registration ───────────────────────────────────────────────────────────

  /// @brief Registers a new daily repeating event.
  /// @param hour   0–23
  /// @param minute 0–59
  /// @param second 0–59
  /// @param cb     Callback to invoke each day at the given time.
  /// @return       Stable event ID on success, INVALID_EVENT_ID on failure.
  ///               Store this ID if you need to cancel the event later.
  uint16_t add_daily_event(uint8_t hour, uint8_t minute, uint8_t second, ScheduledEvent::CallbackFunction cb);

  /// @brief Registers a new one-time event.
  /// @param evt_at Absolute timestamp (seconds since Unix epoch) when the event should fire.
  /// @param cb     Callback to invoke when the time is reached or passed.
  /// @return       Stable event ID on success, INVALID_EVENT_ID on failure.
  ///               Store this ID if you need to cancel the event later.
  uint16_t add_onetime_event(time_t evt_at, ScheduledEvent::CallbackFunction cb);

  // ── Cancellation ───────────────────────────────────────────────────────────

  /// @brief Cancels a previously registered event by its ID.
  ///
  /// Safe to call from inside a callback (the event is marked for deferred
  /// removal and purged at the end of the current process_events() pass).
  /// Safe to call with a stale or already-removed ID — it simply returns false.
  ///
  /// @param id     The ID returned by add_daily_event() (or other add_*()).
  /// @return       True if the event was found and cancelled, false otherwise.
  bool remove_event(uint16_t id);

  // ── Main loop integration ─────────────────────────────────────────────────

  /// @brief Processes all events against the current time, then purges any
  ///        that have been marked for removal.
  /// @param time Current wall-clock time from your RTC/SNTP source.
  void process_events(const ESPTime &time);

  // ── Diagnostics ───────────────────────────────────────────────────────────
  size_t event_count() const { return this->count_; }

 private:
  std::array<ScheduledEvent, MAX_EVENTS> events_;
  size_t count_{0};
  // ── ID generation ─────────────────────────────────────────────────────────
  // Simple monotonic counter. uint16_t gives 65535 unique IDs (0xFFFF is reserved
  // as INVALID_EVENT_ID). For a system that adds/removes a handful of events
  // over its lifetime this will never wrap.
  uint16_t id_counter_{0};

  uint16_t next_id_() {
    // Skip the sentinel value
    if (this->id_counter_ == INVALID_EVENT_ID) {
      this->id_counter_ = 0;
    }
    return this->id_counter_++;
  }
};

}  // namespace sprinklify
}  // namespace esphome
