// Labeled-upper projection, dispatch metadata, frozen-plan replay.
// Split from the original br_lacam_upper.cpp.

#include "br_lacam_upper_internal.hpp"

using namespace br_detail;

namespace {

bool same_transition(
    const BRUpperTransition& a, const BRUpperTransition& b)
{
  if (a.transfers.size() != b.transfers.size()) return false;
  for (size_t i = 0; i < a.transfers.size(); ++i)
    if (a.transfers[i].stable_shelf !=
            b.transfers[i].stable_shelf ||
        !same_transfer(
            a.transfers[i].transfer,
            b.transfers[i].transfer))
      return false;
  return true;
}

std::optional<std::vector<BRUpperConstraintEntry>>
constraints_for_transition(
    const BRLabeledUpperState& state,
    const BRUpperTransition& transition,
    const Deadline* deadline, bool* cutoff)
{
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  if (expired()) return std::nullopt;
  const size_t shelf_count =
      state.target_pos.size() + state.anonymous_pos.size();
  std::vector<BRUpperConstraintEntry> constraints;
  constraints.reserve(shelf_count);
  for (size_t index = 0; index < shelf_count; ++index) {
    if (expired()) return std::nullopt;
    constraints.push_back(
        BRUpperConstraintEntry::make_wait(
            handle_at(state, (int)index)));
  }

  std::vector<uint8_t> seen(shelf_count, 0);
  for (const auto& applied : transition.transfers) {
    if (expired()) return std::nullopt;
    const int index = handle_index(state, applied.stable_shelf);
    if (index < 0 || seen[index]) return std::nullopt;
    seen[index] = 1;
    constraints[index] =
        BRUpperConstraintEntry::make_transfer(
            applied.stable_shelf, applied.transfer);
  }
  return constraints;
}

}  // namespace

UpperSignature project_labeled_upper_state(
    const BRLabeledUpperState& state)
{
  UpperSignature out;
  out.target_pos = state.target_pos;
  out.anon_pos = state.anonymous_pos;
  std::sort(out.anon_pos.begin(), out.anon_pos.end());
  return out;
}

CanonicalDispatchMetadata canonical_dispatch_metadata(
    const BRLabeledUpperState& state, const std::vector<int>& tau,
    const AppliedUpperTransfer& transfer)
{
  CanonicalDispatchMetadata out;
  const auto shelf = transfer.stable_shelf;
  if (shelf.kind != UpperShelfHandle::Kind::TARGET ||
      shelf.stable_id < 0 ||
      shelf.stable_id >= (int)state.target_pos.size() ||
      shelf.stable_id >= (int)tau.size() ||
      transfer.transfer.route.empty() ||
      transfer.transfer.route.front() != state.position(shelf) ||
      transfer.transfer.endpoint != tau[shelf.stable_id])
    return out;
  out.roots.push_back(
      RootDemand{shelf.stable_id, tau[shelf.stable_id]});
  out.priority = 1;
  return out;
}

bool replay_frozen_task_plan(
    const DDInstance& ins, const FrozenTaskPlan& plan,
    const std::vector<int>& tau,
    BRLabeledUpperState* final_state,
    const Deadline* deadline, bool* cutoff)
{
  if (cutoff != nullptr) *cutoff = false;
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  if (expired()) return false;
  if (!valid_grounded_state(
          ins, plan.initial_labeled_state) ||
      tau.size() != plan.initial_labeled_state.target_pos.size())
    return false;
  for (size_t target = 0; target < tau.size(); ++target) {
    if (expired()) return false;
    if (target >= ins.target_goal_sets.size()) return false;
    const auto& goals = ins.target_goal_sets[target];
    if (!std::binary_search(
            goals.begin(), goals.end(), tau[target]))
      return false;
  }

  BRLabeledUpperState state = plan.initial_labeled_state;
  for (size_t wave_index = 0;
       wave_index < plan.waves.size(); ++wave_index) {
    if (expired()) return false;
    const auto& wave = plan.waves[wave_index];
    if (wave.expected_before != state || wave.tasks.empty())
      return false;

    const size_t shelf_count =
        state.target_pos.size() + state.anonymous_pos.size();
    std::vector<BRUpperConstraintEntry> constraints;
    constraints.reserve(shelf_count);
    for (size_t index = 0; index < shelf_count; ++index) {
      if (expired()) return false;
      constraints.push_back(
          BRUpperConstraintEntry::make_wait(
              handle_at(state, (int)index)));
    }
    std::vector<uint8_t> seen(shelf_count, 0);

    for (size_t ordinal = 0;
         ordinal < wave.tasks.size(); ++ordinal) {
      if (expired()) return false;
      const auto& frozen = wave.tasks[ordinal];
      const int index =
          handle_index(state, frozen.stable_shelf);
      if (index < 0 || seen[index] ||
          frozen.id != FrozenTaskId{wave_index, ordinal} ||
          frozen.wave != wave_index)
        return false;
      seen[index] = 1;

      const int source = state.position(frozen.stable_shelf);
      const auto& transfer = frozen.task.transfer;
      if (transfer.route.size() < 2 ||
          transfer.route.front() != source ||
          transfer.route.back() != transfer.endpoint ||
          frozen.task.id.shelf !=
              epoch_selector(state, frozen.stable_shelf) ||
          frozen.task.id.from != source ||
          frozen.task.id.to != transfer.route[1])
        return false;

      const AppliedUpperTransfer applied{
          frozen.stable_shelf, transfer};
      const auto metadata =
          canonical_dispatch_metadata(state, tau, applied);
      if (frozen.task.roots != metadata.roots ||
          frozen.task.priority != metadata.priority)
        return false;

      constraints[index] =
          BRUpperConstraintEntry::make_transfer(
              frozen.stable_shelf, transfer);
    }

    bool exact_cutoff = false;
    const auto successor = validate_complete_upper_action(
        ins, state, constraints, deadline, &exact_cutoff);
    if (exact_cutoff) {
      if (cutoff != nullptr) *cutoff = true;
      return false;
    }
    if (!successor.has_value() ||
        successor->state != wave.expected_after ||
        successor->transition.transfers.size() !=
            wave.tasks.size())
      return false;
    state = successor->state;
  }

  if (expired()) return false;
  if (state.target_pos != tau) return false;
  if (final_state != nullptr) *final_state = state;
  return true;
}

std::optional<FrozenTaskPlan> compile_frozen_task_plan(
    const DDInstance& ins, const BRUpperSearchResult& upper,
    const std::vector<int>& tau,
    const Deadline* deadline, bool* cutoff)
{
  if (cutoff != nullptr) *cutoff = false;
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  if (expired()) return std::nullopt;
  if (!upper.solved() || upper.states.empty() ||
      upper.states.size() != upper.transitions.size() + 1 ||
      tau.size() != upper.states.front().target_pos.size())
    return std::nullopt;

  FrozenTaskPlan plan;
  plan.initial_labeled_state = upper.states.front();
  plan.waves.reserve(upper.transitions.size());
  for (size_t wave_index = 0;
       wave_index < upper.transitions.size(); ++wave_index) {
    if (expired()) return std::nullopt;
    const auto& before = upper.states[wave_index];
    const auto& claimed_after = upper.states[wave_index + 1];
    const auto& transition = upper.transitions[wave_index];
    if (transition.transfers.empty()) return std::nullopt;

    const auto constraints = constraints_for_transition(
        before, transition, deadline, cutoff);
    if (!constraints.has_value()) return std::nullopt;
    bool exact_cutoff = false;
    const auto exact = validate_complete_upper_action(
        ins, before, *constraints, deadline, &exact_cutoff);
    if (exact_cutoff) {
      if (cutoff != nullptr) *cutoff = true;
      return std::nullopt;
    }
    if (!exact.has_value() ||
        exact->state != claimed_after ||
        !same_transition(exact->transition, transition))
      return std::nullopt;

    FrozenTaskWave wave;
    wave.expected_before = before;
    wave.expected_after = claimed_after;
    wave.tasks.reserve(transition.transfers.size());
    for (size_t ordinal = 0;
         ordinal < transition.transfers.size(); ++ordinal) {
      if (expired()) return std::nullopt;
      const auto& applied = transition.transfers[ordinal];
      const int source = before.position(applied.stable_shelf);
      if (source < 0 || applied.transfer.route.size() < 2 ||
          applied.transfer.route.front() != source)
        return std::nullopt;

      const auto metadata =
          canonical_dispatch_metadata(before, tau, applied);
      FrozenShelfTask frozen;
      frozen.id = FrozenTaskId{wave_index, ordinal};
      frozen.stable_shelf = applied.stable_shelf;
      frozen.wave = wave_index;
      frozen.task.id = TaskId{
          epoch_selector(before, applied.stable_shelf),
          source, applied.transfer.route[1]};
      frozen.task.roots = metadata.roots;
      frozen.task.priority = metadata.priority;
      frozen.task.transfer = applied.transfer;
      wave.tasks.push_back(std::move(frozen));
    }
    plan.waves.push_back(std::move(wave));
  }

  bool replay_cutoff = false;
  if (!replay_frozen_task_plan(
          ins, plan, tau, nullptr, deadline, &replay_cutoff)) {
    if (replay_cutoff && cutoff != nullptr) *cutoff = true;
    return std::nullopt;
  }
  return plan;
}
