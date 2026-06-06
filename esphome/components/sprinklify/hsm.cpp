#include "hsm.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sprinklify {

static const char *const TAG = "sprinklify.hsm";

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------

/// @brief Registers a state along with its event handler and its parent state.
/// @param state Unique ID of the state to register.
/// @param handler Callback function for handling events.
/// @param parent Parent state ID.
/// @return True if successful, false otherwise.
bool HSM::register_state(HSMStateType state, StateHandler handler, HSMStateType parent) {
  if (state >= MAX_STATES) {
    ESP_LOGE(TAG, "register_state: state %u exceeds MAX_STATES (%u)", state, MAX_STATES);
    this->registration_error_ = true;
    return false;
  }

  auto &sd = STATE_(state);
  sd.handler = std::move(handler);

  if (parent == STATE_INVALID) {
    if (state != this->root_state_) {
      ESP_LOGE(TAG, "register_state: state %u has no parent but is not the root state (%u)", state, this->root_state_);
      this->registration_error_ = true;
      return false;
    }
    sd.parent_state = STATE_INVALID;
    sd.level = 0;
    sd.registered = true;
    ESP_LOGD(TAG, "State %u registered as ROOT", state);
    return true;
  }

  sd.parent_state = parent;

  // Calculate hierarchy level by climbing from parent to root.
  HSMStateType p = parent;
  uint8_t level = 1;
  while (p != this->root_state_ && level < MAX_HIERARCHY_DEPTH) {
    if (p >= MAX_STATES || !STATE_(p).registered) {
      ESP_LOGE(TAG,
               "register_state: state %u — parent chain broken at %u "
               "(not registered or out of range). Register parents before children.",
               state, p);
      this->registration_error_ = true;
      return false;
    }
    p = STATE_(p).parent_state;
    ++level;
  }

  sd.level = level;
  sd.registered = true;
  ESP_LOGD(TAG, "State %u registered at level %u with parent %u", state, level, parent);
  return true;
}

// ---------------------------------------------------------------------------
// Start
// ---------------------------------------------------------------------------

/// @brief Starts the state machine. It “drills into” initial transitions triggered by EVT_INIT.
/// The implementation follows the UML convention that when a composite state is entered,
/// its initial transition (if any) is taken without executing exit actions.
bool HSM::start() {
  if (this->initialized_) {
    ESP_LOGW(TAG, "start() called on an already-running HSM — ignored");
    return false;
  }
  if (this->registration_error_) {
    ESP_LOGE(TAG, "start() called but there were state registration errors");
    return false;
  }
  if (!STATE_(this->root_state_).registered) {
    ESP_LOGE(TAG, "start() called but root state %u is not registered", this->root_state_);
    return false;
  }

  this->current_state_ = this->root_state_;
  ESP_LOGD(TAG, "HSM starting — entering root state %u", this->root_state_);
  DO_ENTRY(this->root_state_);

  this->follow_init_transitions_();

  this->initialized_ = true;
  ESP_LOGI(TAG, "HSM started — initial state %u", this->current_state_);
  return true;
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------

/// @brief Handles the state machine. This should be called from loop.
void HSM::process() {
  if (!this->initialized_)
    return;

  // Handle transitions first - they change the state context
  if (this->transition_pending_) {
    this->execute_transition_();
    return;
  }

  // Process one event per call
  if (this->queue_head_ != this->queue_tail_) {
    HSMEventType event = this->event_queue_[this->queue_tail_];
    this->queue_tail_ = (this->queue_tail_ + 1) & (EVENT_QUEUE_SIZE - 1);
    ESP_LOGV(TAG, "Dispatching event %u in state %u", event, this->current_state_);
    this->dispatch_event_(event);
  }
}

// ---------------------------------------------------------------------------
// Event dispatch — bubble up through ancestor chain
// ---------------------------------------------------------------------------

/// @brief Dispatches an event to the active state and, if necessary, bubbles
/// the event up to parent states until it is handled.
/// @param event The event to be processed
void HSM::dispatch_event_(HSMEventType event) {
  HSMStateType state = this->current_state_;

  while (state != this->root_state_) {
    this->dispatching_state_ = state;
    StateResult result = STATE_(state).handler(event);
    this->dispatching_state_ = STATE_INVALID;

    if (result == RET_HANDLED || result == RET_TRANSITION || result == RET_IGNORED) {
      ESP_LOGV(TAG, "Event %u handled by state %u (result %u)", event, state, result);
      return;
    }

    // RET_UNHANDLED — try parent
    ESP_LOGV(TAG, "Event %u unhandled in state %u — bubbling to parent %u", event, state, STATE_(state).parent_state);
    state = STATE_(state).parent_state;
  }

  // Root state — final handler, result is informational only
  this->dispatching_state_ = state;
  StateResult result = STATE_(state).handler(event);
  this->dispatching_state_ = STATE_INVALID;
  ESP_LOGV(TAG, "Event %u reached root state %u (result %u)", event, state, result);
}

// ---------------------------------------------------------------------------
// Transition execution
// ---------------------------------------------------------------------------

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
void HSM::execute_transition_() {
  const HSMStateType target = this->target_state_;
  const HSMStateType source = this->transition_source_state_;
  this->transition_source_state_ = STATE_INVALID;

  ESP_LOGD(TAG, "Transition: state %u → %u (source %u)", this->current_state_, target, source);

  // -------------------------------------------------------------------------
  // Self-transition: exit current leaf up to and including source, re-enter.
  // -------------------------------------------------------------------------
  if (source == target) {
    HSMStateType s = this->current_state_;

    while (true) {
      ESP_LOGD(TAG, "  exit %u (self-transition)", s);
      this->current_state_ = s;
      DO_EXIT(s);

      if (s == source)
        break;

      s = STATE_(s).parent_state;
      if (s == STATE_INVALID) {
        ESP_LOGE(TAG, "execute_transition: self-transition source %u not found in ancestor chain", source);
        this->transition_pending_ = false;
        return;
      }
    }

    ESP_LOGD(TAG, "  entry %u (self-transition re-entry)", target);
    this->current_state_ = target;
    DO_ENTRY(target);
    this->follow_init_transitions_();
    return;
  }

  // -------------------------------------------------------------------------
  // Normal LCA transition.
  // -------------------------------------------------------------------------
  int path_length = 0;
  HSMStateType s1 = this->current_state_;
  HSMStateType s2 = target;
  int level1 = STATE_(s1).level;
  int level2 = STATE_(s2).level;

  // Exit source side down to the LCA level
  while (level1 > level2) {
    ESP_LOGD(TAG, "  exit %u", s1);
    this->current_state_ = s1;
    DO_EXIT(s1);
    s1 = STATE_(s1).parent_state;
    --level1;
  }

  // Collect target entry path down to LCA level (no exits on target side)
  while (level2 > level1) {
    this->transition_path_[path_length++] = s2;
    s2 = STATE_(s2).parent_state;
    --level2;
  }

  // Walk both sides up until LCA found, exiting source side
  while (s1 != s2) {
    ESP_LOGD(TAG, "  exit %u", s1);
    this->current_state_ = s1;
    DO_EXIT(s1);
    this->transition_path_[path_length++] = s2;
    s1 = STATE_(s1).parent_state;
    s2 = STATE_(s2).parent_state;
  }

  ESP_LOGD(TAG, "  LCA is state %u", s1);

  // Enter from LCA down to target (path is stored reversed)
  for (int i = path_length - 1; i >= 0; --i) {
    ESP_LOGD(TAG, "  entry %u", this->transition_path_[i]);
    this->current_state_ = this->transition_path_[i];
    DO_ENTRY(this->transition_path_[i]);
  }

  this->current_state_ = target;
  this->follow_init_transitions_();
}

// ---------------------------------------------------------------------------
// INIT chain — drill into composite states after entry
// ---------------------------------------------------------------------------

/// @brief Processes automatic INIT transitions after entering a state.
/// If the state's INIT handler returns TRANSITION, then that means an automatic (initial) transition is desired.
void HSM::follow_init_transitions_() {
  while (true) {
    StateResult result = STATE_(this->current_state_).handler(ReservedEvents::EVT_INIT);
    if (result != RET_TRANSITION)
      break;
    ESP_LOGD(TAG, "  INIT from %u → %u", this->current_state_, this->target_state_);
    apply_entry_path_(this->current_state_, this->target_state_);
    this->current_state_ = this->target_state_;
  }
  // Clear transition flag
  this->transition_pending_ = false;
  this->transition_source_state_ = STATE_INVALID;
  ESP_LOGD(TAG, "  settled in state %u", this->current_state_);
}

// ---------------------------------------------------------------------------
// Entry path helper — enters states from LCA down to target without exits
// ---------------------------------------------------------------------------

/// @brief A helper used in the initial state traversal to build an entry path
/// without calling exit events. This is used when processing automatic (initial) transitions
/// within composite states.
void HSM::apply_entry_path_(HSMStateType from_state, HSMStateType to_state) {
  uint8_t path_length = 0;

  // Build path from target state up to (but not including) the current state.
  for (HSMStateType t = to_state; t != from_state; t = STATE_(t).parent_state) {
    this->transition_path_[path_length++] = t;
    if (path_length >= MAX_HIERARCHY_DEPTH) {
      ESP_LOGW(TAG, "apply_entry_path: hierarchy depth exceeded — truncating");
      break;
    }
  }

  // Reverse the path to perform entry from top to bottom.
  // Set current_state_ before each ENTRY so the handler sees itself as current.
  // The caller (follow_init_transitions) will overwrite current_state_ with the
  // final target immediately after this call, so the last write here is transient.
  for (int i = path_length - 1; i >= 0; --i) {
    ESP_LOGV(TAG, "  entry %u (init path)", this->transition_path_[i]);
    this->current_state_ = this->transition_path_[i];
    DO_ENTRY(this->transition_path_[i]);
  }
}

}  // namespace sprinklify
}  // namespace esphome
