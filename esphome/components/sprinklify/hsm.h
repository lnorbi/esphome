#pragma once

#include <functional>
#include <array>
#include <cstddef>

namespace esphome {
namespace sprinklify {

using HSMStateType = uint8_t;
using HSMEventType = uint8_t;

constexpr HSMStateType MAX_STATES = 16;
constexpr HSMStateType STATE_INVALID = 0xFF;
constexpr uint8_t MAX_HIERARCHY_DEPTH = 6;
constexpr uint8_t EVENT_QUEUE_SIZE = 8;  // must be a power of 2

#define STATE_(state) this->states_[state]
#define DO_ENTRY(state) (this->states_[(state)].handler(ReservedEvents::EVT_ENTRY))
#define DO_EXIT(state) (this->states_[(state)].handler(ReservedEvents::EVT_EXIT))

// Reserved events. User events should start from HSM_FIRST_USER_EVENT!
enum ReservedEvents : HSMEventType {
  EVT_ENTRY = 0,
  EVT_EXIT,
  EVT_INIT,
  HSM_FIRST_USER_EVENT,
};

// State handler return codes
enum StateResult : uint8_t {
  RET_UNHANDLED = 0,
  RET_HANDLED,
  RET_TRANSITION,
  RET_IGNORED,
};

using StateHandler = std::function<StateResult(HSMEventType)>;

class HSM {
 public:
  explicit HSM(HSMStateType root_state)
      : root_state_(root_state), initialized_(false), transition_pending_(false), queue_head_(0), queue_tail_(0) {}

  // --- Public API — declarations; bodies in hsm.cpp ---

  bool register_state(HSMStateType state, StateHandler handler, HSMStateType parent = STATE_INVALID);
  bool start();
  void process();

  // --- Hot-path inlines ---

  /// @brief Posts an event to the circular event queue.
  /// Safe to call from sensor callbacks — drops silently if the queue is full.
  inline void post_event(HSMEventType event) {
    uint8_t next_head = (this->queue_head_ + 1) & (EVENT_QUEUE_SIZE - 1);
    if (next_head != this->queue_tail_) {
      this->event_queue_[this->queue_head_] = event;
      this->queue_head_ = next_head;
    }
  }

  /// @brief Records a pending transition and returns RET_TRANSITION.
  /// Must be called from inside a state handler. The transition executes on
  /// the next process() call, after the current handler returns.
  inline StateResult transition_to(HSMStateType target_state) {
    this->target_state_ = target_state;
    this->transition_source_state_ =
        this->dispatching_state_ != STATE_INVALID ? this->dispatching_state_ : this->current_state_;
    this->transition_pending_ = true;
    return RET_TRANSITION;
  }

  inline HSMStateType current_state() const { return this->current_state_; }
  inline bool is_ready() const { return this->initialized_; }
  inline bool is_idle() const { return this->queue_head_ == this->queue_tail_ && !this->transition_pending_; }

 private:
  struct StateDescriptor {
    StateHandler handler;
    HSMStateType parent_state{STATE_INVALID};
    uint8_t level{0};
    bool registered{false};
  };

  void dispatch_event_(HSMEventType event);
  void execute_transition_();
  void follow_init_transitions_();
  void apply_entry_path_(HSMStateType from_state, HSMStateType to_state);

  std::array<HSMEventType, EVENT_QUEUE_SIZE> event_queue_{};
  uint8_t queue_head_{0};
  uint8_t queue_tail_{0};

  StateDescriptor states_[MAX_STATES];
  HSMStateType current_state_{STATE_INVALID};
  HSMStateType target_state_{STATE_INVALID};
  HSMStateType root_state_;
  HSMStateType transition_source_state_{STATE_INVALID};
  HSMStateType dispatching_state_{STATE_INVALID};

  HSMStateType transition_path_[MAX_HIERARCHY_DEPTH]{};

  bool transition_pending_{false};
  bool initialized_{false};
};

}  // namespace sprinklify
}  // namespace esphome
