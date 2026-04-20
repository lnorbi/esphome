#include "event_scheduler.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sprinklify {

static const char *const SCHEDULER_TAG = "sprinklify.scheduler";

uint16_t EventScheduler::add_daily_event(uint8_t hour, uint8_t minute, uint8_t second,
                                         ScheduledEvent::CallbackFunction cb) {
  if (hour > 23 || minute > 59 || second > 59) {
    ESP_LOGW(SCHEDULER_TAG,
             "add_daily_event: invalid time %02" PRIu8 ":%02" PRIu8 ":%02" PRIu8 " — event not registered", hour,
             minute, second);
    return INVALID_EVENT_ID;
  }
  if (this->count_ >= MAX_EVENTS) {
    ESP_LOGW(SCHEDULER_TAG, "add_daily_event: scheduler full (%u/%u)", this->count_, MAX_EVENTS);
    return INVALID_EVENT_ID;
  }

  const uint16_t id = this->next_id_();
  this->events_[this->count_++].set_daily(id, hour, minute, second, std::move(cb));

  ESP_LOGD(SCHEDULER_TAG,
           "Daily event #%" PRIu16 " registered for %02" PRIu8 ":%02" PRIu8 ":%02" PRIu8 " (%u/%u slots used)", id,
           hour, minute, second, this->count_, MAX_EVENTS);
  return id;
}

uint16_t EventScheduler::add_onetime_event(time_t evt_at, ScheduledEvent::CallbackFunction cb) {
  if (this->count_ >= MAX_EVENTS) {
    ESP_LOGW(SCHEDULER_TAG, "add_onetime_event: scheduler full (%u/%u)", this->count_, MAX_EVENTS);
    return INVALID_EVENT_ID;
  }

  uint16_t id = this->next_id_();
  this->events_[this->count_++].set_one_time(id, evt_at, std::move(cb));

  ESP_LOGD(SCHEDULER_TAG, "One-time event #%" PRIu16 " registered for timestamp %lu (%u/%u slots used)", id,
           static_cast<unsigned long>(evt_at), this->count_, MAX_EVENTS);
  return id;
}

bool EventScheduler::remove_event(uint16_t id) {
  if (id == INVALID_EVENT_ID) {
    ESP_LOGW(SCHEDULER_TAG, "remove_event: called with INVALID_EVENT_ID — ignored");
    return false;
  }

  for (size_t i = 0; i < this->count_; i++) {
    if (this->events_[i].id == id) {
      // Mark for deferred removal rather than erasing in-place.
      // This makes remove_event() safe to call from within a callback
      // that is itself being invoked by process_events().
      this->events_[i].should_remove = true;
      ESP_LOGD(SCHEDULER_TAG, "Event #%" PRIu16 " cancelled", id);
      return true;
    }
  }

  ESP_LOGW(SCHEDULER_TAG, "remove_event: event #%" PRIu16 " not found (already removed?)", id);
  return false;
}

void EventScheduler::process_events(const ESPTime &time) {
  for (auto &event : this->events_) {
    event.process(time);
  }

  // Compact the array: shift live events forward over removed slots.
  // O(n) single pass, no allocations.
  size_t write = 0;
  for (size_t read = 0; read < this->count_; read++) {
    if (!this->events_[read].should_remove)
      this->events_[write++] = this->events_[read];
  }
  this->count_ = write;
}

}  // namespace sprinklify
}  // namespace esphome
