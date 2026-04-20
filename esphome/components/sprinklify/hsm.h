#pragma once

#include <functional>
#include <array>
#include <optional>
#include <cstddef>
// #include <cstdint>

namespace esphome {
namespace sprinklify {

using HSMStateType = uint8_t;
using HSMEventType = uint8_t;

// Configuration - adjust these for your needs
constexpr HSMStateType MAX_STATES = 16;       // Maximum number of states
constexpr HSMStateType STATE_INVALID = 0xFF;  // Special code for Invalid state
constexpr uint8_t MAX_HIERARCHY_DEPTH = 6;    // Maximum nesting depth
constexpr uint8_t EVENT_QUEUE_SIZE = 8;       // Event queue size (power of 2)

// constexpr EventType FIRST_USER_EVENT = 3;   // First available user event number

#define STATE_(state) this->states_[state]

// Helper macros to standardize states' entry and exit calls.
// They wrap the call to a state's event handler.
#define DO_ENTRY(state) (this->states_[(state)].handler(ReservedEvents::EVT_ENTRY))
#define DO_EXIT(state) (this->states_[(state)].handler(ReservedEvents::EVT_EXIT))

// Reserved events. User events should start from USER_EVENT!
enum ReservedEvents : HSMEventType {
  EVT_ENTRY = 0,
  EVT_EXIT,
  EVT_INIT,
  HSM_FIRST_USER_EVENT,
};

// State return codes
enum StateResult : uint8_t {
  RET_UNHANDLED = 0,  // Event was unhandled, try parent
  RET_HANDLED,        // Event was handled
  RET_TRANSITION,     // Transition requested
  RET_IGNORED,        // Event was ignored, stop bubbling up
};

// A state handler function (using std::function, so that lambdas can be stored).
using StateHandler = std::function<StateResult(HSMEventType)>;

class HSM {
 public:
  explicit HSM(HSMStateType root_state)
      : root_state_(root_state), initialized_(false), transition_pending_(false), queue_head_(0), queue_tail_(0) {}

  /// @brief Registers a state along with its event handler and its parent state.
  /// @param state Unique ID of the state to register.
  /// @param handler Callback function for handling events.
  /// @param parent Parent state ID.
  /// @return True if successful, false otherwise.
  bool register_state(HSMStateType state, StateHandler handler, HSMStateType parent = STATE_INVALID) {
    if (state >= MAX_STATES)
      return false;

    auto &sd = STATE_(state);
    sd.handler = std::move(handler);
    // Use root_state as parent if no other parent given.
    HSMStateType p = (parent == STATE_INVALID) ? this->root_state_ : parent;
    sd.parent_state = p;

    // Calculate hierarchy level by climbing up from the parent.
    uint8_t level = 0;
    while (p != this->root_state_ && level < MAX_HIERARCHY_DEPTH) {
      if (p >= MAX_STATES || !STATE_(p).registered)
        return false;
      p = STATE_(p).parent_state;
      ++level;
    }
    sd.level = level;
    sd.registered = true;
    return true;
  }

  /// @brief Starts the state machine. It “drills into” initial transitions triggered by EVT_INIT.
  /// The implementation follows the UML convention that when a composite state is entered,
  /// its initial transition (if any) is taken without executing exit actions.
  bool start() {
    // TODO Maybe we don't need safety checks here
    if (this->initialized_ || !STATE_(this->root_state_).registered)
      return false;

    this->current_state_ = this->root_state_;

    // Enter the root state.
    DO_ENTRY(this->root_state_);

    // Process INIT transitions as long as the current state requests to auto-transition.
    this->follow_init_transitions();

    this->initialized_ = true;
    return true;
  }

  /// @brief Posts an event to the event queue
  /// @param event The event to be posted
  inline void post_event(HSMEventType event) {
    uint8_t next_head = (this->queue_head_ + 1) & (EVENT_QUEUE_SIZE - 1);
    if (next_head != this->queue_tail_) {
      this->event_queue_[this->queue_head_] = event;
      this->queue_head_ = next_head;
    }
  }

  /// @brief Request state transition
  /// @param target_state Target state to transition to
  // inline void transition_to(StateType target_state) {
  //   target_state_ = target_state;
  //   transition_pending_ = true;
  // }

  inline StateResult transition_to(HSMStateType target_state) {
    this->target_state_ = target_state;
    this->transition_source_state_ =
        this->dispatching_state_ != STATE_INVALID ? this->dispatching_state_ : this->current_state_;
    this->transition_pending_ = true;
    return RET_TRANSITION;
  }

  /// @brief Handles the state machine. This should be called from loop.
  void process() {
    if (!this->initialized_)
      return;

    // Handle transitions first - they change the state context
    if (this->transition_pending_) {
      this->execute_transition();
      return;
    }

    // Process one event per call
    if (this->queue_head_ != this->queue_tail_) {
      HSMEventType event = this->event_queue_[queue_tail_];
      this->queue_tail_ = (this->queue_tail_ + 1) & (EVENT_QUEUE_SIZE - 1);
      this->dispatch_event(event);
    }
  }

  /// @brief Gets the current state
  /// @return Current state
  inline HSMStateType current_state() const { return this->current_state_; }

  /// @brief Checks if the HSM is initialized & ready.
  inline bool is_ready() const { return this->initialized_; }

  /// @brief Returns true if the state machine is idle (i.e., no pending events or transitions).
  inline bool is_idle() const { return this->queue_head_ == this->queue_tail_ && !this->transition_pending_; }

 private:
  // Lightweight state descriptor
  struct StateDescriptor {
    StateHandler handler;
    HSMStateType parent_state;
    uint8_t level = 0;  // Hierarchy level (0 = root)
    bool registered = false;
  };

  /// @brief Dispatches an event to the active state and, if necessary, bubbles the event up
  /// to parent states until it is handled.
  /// @param event The event to be processed
  void dispatch_event(HSMEventType event) {
    HSMStateType state = this->current_state_;

    while (state != this->root_state_) {
      this->dispatching_state_ = state;
      StateResult result = STATE_(state).handler(event);
      this->dispatching_state_ = STATE_INVALID;

      if (result != RET_UNHANDLED)
        return;

      state = STATE_(state).parent_state;
    }

    // Final check for root state
    this->dispatching_state_ = state;
    STATE_(state).handler(event);
    this->dispatching_state_ = STATE_INVALID;
  }

  /// @brief Executes a transition using this helper's embedded-HSM transition policy.
  ///
  /// Transition policy:
  ///
  /// 1. Local/LCA transition by default
  ///    For normal transitions, the machine exits from the active leaf up to,
  ///    but not including, the lowest common ancestor of the active leaf and
  ///    the target. It then enters the target path from that ancestor downward.
  ///
  ///    This means ancestor targets are not exited/re-entered. Example:
  ///
  ///      Active: A / A1 / A1a
  ///      Source: A1a
  ///      Target: A
  ///
  ///    Result:
  ///
  ///      exit A1a
  ///      exit A1
  ///      INIT A
  ///      ...
  ///
  ///    A itself remains active.
  ///
  /// 2. Explicit external self-transition
  ///    If the state that requested the transition is also the target state,
  ///    this is treated as a deliberate restart of that state. The machine exits
  ///    the active leaf up to and including that source/target state, then
  ///    re-enters it and follows its INIT chain.
  ///
  ///    Example:
  ///
  ///      Active: A / A1 / A1a
  ///      Source: A
  ///      Target: A
  ///
  ///    Result:
  ///
  ///      exit A1a
  ///      exit A1
  ///      exit A
  ///      entry A
  ///      INIT A
  ///      ...
  ///
  /// This is intentionally not full UML external-transition semantics. It is a
  /// local-by-default HSM dialect with one explicit external case: self-restart.
  void execute_transition() {
    const HSMStateType target = this->target_state_;
    const HSMStateType source = this->transition_source_state_;

    // Clear the stored source now; this transition consumes it.
    this->transition_source_state_ = STATE_INVALID;

    // ---------------------------------------------------------------------------
    // Special case: explicit external self-transition.
    //
    // This is the only transition type in this helper that exits and re-enters
    // the target/source composite itself.
    // ---------------------------------------------------------------------------
    if (source == target) {
      HSMStateType s = this->current_state_;

      // Exit from active leaf up to and including the source state.
      while (true) {
        DO_EXIT(s);

        if (s == source)
          break;

        s = STATE_(s).parent_state;

        // Defensive guard. In a valid HSM transition, source must be an ancestor
        // of current_state_. If not, the registration/dispatch/source tracking is wrong.
        if (s == STATE_INVALID) {
          this->transition_pending_ = false;
          return;
        }
      }
      // Re-enter the source/target state.
      DO_ENTRY(target);
      this->current_state_ = target;

      // If target is composite, drill down through its initial transition(s).
      this->follow_init_transitions();
      return;
    }

    // ---------------------------------------------------------------------------
    // Normal non-self transition.
    //
    // Default case: local/LCA transition.
    // An LCA algorithm:
    // - exit from current leaf until LCA with target
    // - build target entry path while climbing target toward LCA
    // - enter from top to bottom
    // - then process INIT transitions
    // ---------------------------------------------------------------------------

    // Find common ancestor and build transition path, also exiting the source states as we go
    int path_length = 0;
    HSMStateType s1 = this->current_state_;
    HSMStateType s2 = target;
    int level1 = STATE_(s1).level;
    int level2 = STATE_(s2).level;

    // Bring both states to same level
    while (level1 > level2) {
      DO_EXIT(s1);
      s1 = STATE_(s1).parent_state;
      --level1;
    }
    while (level2 > level1) {
      this->transition_path_[path_length++] = s2;
      s2 = STATE_(s2).parent_state;
      --level2;
    }
    // Walk both sides upward until common ancestor found, while exiting the source
    while (s1 != s2) {
      DO_EXIT(s1);
      this->transition_path_[path_length++] = s2;
      s1 = STATE_(s1).parent_state;
      s2 = STATE_(s2).parent_state;
    }

    // The entry path is stored in reverse order; reverse it to call the correct entry actions.
    // Note: we do NOT yet enter the target state here
    for (int i = path_length - 1; i >= 0; --i) {
      DO_ENTRY(this->transition_path_[i]);
    }

    this->current_state_ = target;
    // Process any automatic (INIT) transitions for composite states.
    this->follow_init_transitions();
  }

  /// @brief Processes automatic INIT transitions after entering a state.
  /// If the state's INIT handler returns TRANSITION, then that means an automatic (initial) transition is desired.
  void follow_init_transitions() {
    while (true) {
      StateResult result = STATE_(this->current_state_).handler(ReservedEvents::EVT_INIT);
      if (result != RET_TRANSITION)
        break;
      // Apply the entry path from the current state to the new target state.
      apply_entry_path(this->current_state_, this->target_state_);
      this->current_state_ = this->target_state_;
    }
    // Clear transition flag
    this->transition_pending_ = false;
    this->transition_source_state_ = STATE_INVALID;
  }

  /// @brief A helper used in the initial state traversal to build an entry path
  /// without calling exit events. This is used when processing automatic (initial) transitions
  /// within composite states.
  void apply_entry_path(HSMStateType from_state, HSMStateType to_state) {
    uint8_t path_length = 0;
    // Build path from target state up to (but not including) the current state.
    for (HSMStateType t = to_state; t != from_state; t = STATE_(t).parent_state) {
      this->transition_path_[path_length++] = t;
      if (path_length >= MAX_HIERARCHY_DEPTH)
        break;  // Safety check for hierarchy overflow.
    }
    // Reverse the path to perform entry from top to bottom.
    for (int i = path_length - 1; i >= 0; --i) {
      DO_ENTRY(this->transition_path_[i]);
    }
  }

  // Event queue (circular buffer)
  std::array<HSMEventType, EVENT_QUEUE_SIZE> event_queue_;
  uint8_t queue_head_ = 0;  // Index of the next event to read
  uint8_t queue_tail_ = 0;  // Index of the next position to write

  // State management
  StateDescriptor states_[MAX_STATES];
  HSMStateType current_state_{STATE_INVALID};
  HSMStateType target_state_{STATE_INVALID};
  HSMStateType root_state_;
  HSMStateType transition_source_state_{STATE_INVALID};
  HSMStateType dispatching_state_{STATE_INVALID};

  // Cache for the transition path (used during state transitions).
  HSMStateType transition_path_[MAX_HIERARCHY_DEPTH];

  bool transition_pending_ = false;
  bool initialized_ = false;
};

}  // namespace sprinklify
}  // namespace esphome
