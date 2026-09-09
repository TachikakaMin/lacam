// Part of carrier_guidance.hpp (internal, src/): Joint transport guidance.
// Chained include; see carrier_guidance.hpp for the module overview.
#pragma once

#include "carrier_custody.hpp"

namespace carrier_detail {
struct JointTransportFrameScore {
  long long predicted_all_targets_ticks = 0;
  long long predicted_work = 0;
  std::vector<int> stable_order;
};

inline JointTransportFrameScore
make_joint_transport_frame_score(
    const std::vector<long long>& planned_completion,
    const std::vector<long long>& residual_completion,
    long long predicted_work, std::vector<int> stable_order)
{
  JointTransportFrameScore out;
  for (const long long completion : planned_completion)
    out.predicted_all_targets_ticks =
        std::max(out.predicted_all_targets_ticks, completion);
  for (const long long completion : residual_completion)
    out.predicted_all_targets_ticks =
        std::max(out.predicted_all_targets_ticks, completion);
  out.predicted_work = predicted_work;
  out.stable_order = std::move(stable_order);
  return out;
}

inline bool better_joint_transport_frame(
    const JointTransportFrameScore& candidate,
    const JointTransportFrameScore& incumbent)
{
  return std::make_tuple(
             candidate.predicted_all_targets_ticks,
             candidate.predicted_work,
             candidate.stable_order) <
         std::make_tuple(
             incumbent.predicted_all_targets_ticks,
             incumbent.predicted_work,
             incumbent.stable_order);
}

struct JointTransportContext {
  const ShelfTaskGraph* graph = nullptr;
  const ExecutionView* execution_view = nullptr;
  const std::vector<int>* rho_ready_index = nullptr;
  const std::vector<DispatchMode>* rho_mode = nullptr;
  const std::vector<int>* tau_guide = nullptr;
};

inline JointTransportGuidance
build_bounded_joint_transport_guidance(
    const DDInstance& ins, const PhysConfig& physical,
    const std::vector<std::optional<Custody>>& custody_by_robot,
    int horizon = 16, int expansions_per_job = 256,
    int frame_budget = 8,
    const JointTransportContext* context = nullptr)
{
  JointTransportGuidance out;
  out.by_robot.resize(ins.n_robots());
  horizon = std::max(1, horizon);
  expansions_per_job = std::max(0, expansions_per_job);
  frame_budget = std::max(1, frame_budget);
  constexpr long long UNAVAILABLE = 1000000LL;

  const ShelfTaskGraph* graph =
      context != nullptr ? context->graph : nullptr;
  const auto critical_tail =
      graph != nullptr
          ? task_critical_tail_ticks(*graph)
          : std::vector<int>();
  LowerDist lower_distance(ins.grid);

  struct Job {
    int robot = -1;
    int source = -1;
    int endpoint = -1;
    int priority = 0;
    int start_tick = 0;
    int approach_ticks = 0;
    int task_index = -1;
    int tail_ticks = 0;
    bool assigned = false;
    TransferId transfer_id;
    std::vector<RootDemand> roots;
    std::vector<int> topology_distance;
  };
  std::vector<Job> jobs;
  std::vector<uint8_t> job_robot(ins.n_robots(), 0);
  std::set<TransferKey> planned_transfers;
  for (size_t robot = 0; robot < custody_by_robot.size() &&
                         robot < ins.n_robots();
       ++robot) {
    if (!custody_by_robot[robot].has_value()) continue;
    const auto& custody = *custody_by_robot[robot];
    if (!custody_physically_valid(
            ins, physical, (int)robot, custody) ||
        !episode_active(custody) ||
        custody_arrived(physical, (int)robot, custody))
      continue;
    Job job;
    job.robot = (int)robot;
    job.source = physical.robots[robot];
    job.endpoint = custody_endpoint(custody);
    job.priority = custody.priority;
    job.task_index = custody.current_task_index.value_or(-1);
    job.transfer_id = custody.transfer_id;
    job.roots = custody.roots;
    if (graph != nullptr &&
        (job.task_index < 0 ||
         job.task_index >= (int)graph->tasks.size())) {
      for (size_t index = 0; index < graph->tasks.size(); ++index)
        if (transfer_key(graph->tasks[index]) ==
            custody.transfer_id.key) {
          job.task_index = (int)index;
          break;
        }
    }
    if (graph != nullptr &&
        job.task_index >= 0 &&
        job.task_index < (int)graph->tasks.size()) {
      if (job.roots.empty())
        job.roots = graph->tasks[job.task_index].roots;
      if (job.task_index < (int)critical_tail.size())
        job.tail_ticks = critical_tail[job.task_index];
    }
    job.topology_distance =
        transport_topology_distance(
            ins, job.source, job.endpoint);
    planned_transfers.insert(job.transfer_id.key);
    jobs.push_back(std::move(job));
    job_robot[robot] = 1;
  }

  const auto task_is_grounded = [&](const ShelfTask& task) {
    if (task.id.shelf.kind == ShelfSelector::Kind::TARGET) {
      const int target = task.id.shelf.value;
      if (target < 0 ||
          target >= (int)physical.target_pos.size() ||
          physical.target_pos[target] != task.id.from)
        return false;
      return std::find(
                 physical.kappa.begin(), physical.kappa.end(),
                 target) == physical.kappa.end();
    }
    return task.id.shelf.value == task.id.from &&
           std::binary_search(
               physical.anon_occ.begin(), physical.anon_occ.end(),
               task.id.from);
  };
  if (graph != nullptr && context != nullptr &&
      context->rho_ready_index != nullptr &&
      context->rho_mode != nullptr) {
    for (size_t robot = 0; robot < ins.n_robots() &&
                           robot < physical.kappa.size() &&
                           robot < context->rho_ready_index->size() &&
                           robot < context->rho_mode->size();
         ++robot) {
      if (job_robot[robot] ||
          physical.kappa[robot] != KAPPA_FREE)
        continue;
      const int task_index =
          (*context->rho_ready_index)[robot];
      const DispatchMode mode = (*context->rho_mode)[robot];
      if (mode == DispatchMode::NONE ||
          task_index < 0 ||
          task_index >= (int)graph->tasks.size())
        continue;
      const auto& task = graph->tasks[task_index];
      if (!task_is_grounded(task)) continue;
      const TransferKey key = transfer_key(task);
      if (planned_transfers.count(key) != 0) continue;
      const int approach =
          lower_distance.dist(
              task.id.from, physical.robots[robot]);
      Job job;
      job.robot = (int)robot;
      job.source = task.id.from;
      job.endpoint = key.endpoint;
      job.priority = task.priority;
      job.task_index = task_index;
      job.tail_ticks =
          task_index < (int)critical_tail.size()
              ? critical_tail[task_index]
              : 0;
      job.assigned = true;
      job.approach_ticks =
          approach < INT_MAX / 4 ? approach : (int)UNAVAILABLE;
      job.start_tick =
          mode == DispatchMode::EXECUTE &&
                  approach < INT_MAX / 4
              ? approach + 1
              : horizon;
      job.transfer_id =
          TransferId{key, (int)robot,
                     phys_config_hash(physical)};
      job.roots = task.roots;
      job.topology_distance =
          transport_topology_distance(
              ins, job.source, job.endpoint);
      planned_transfers.insert(key);
      jobs.push_back(std::move(job));
      job_robot[robot] = 1;
    }
  }
  std::stable_sort(
      jobs.begin(), jobs.end(),
      [](const Job& a, const Job& b) {
        if (a.priority != b.priority)
          return a.priority > b.priority;
        if (a.transfer_id != b.transfer_id)
          return a.transfer_id < b.transfer_id;
        return a.robot < b.robot;
      });

  std::set<RootDemand> represented_roots;
  for (const auto& job : jobs)
    represented_roots.insert(
        job.roots.begin(), job.roots.end());
  std::map<RootDemand, long long> residual_by_root;
  const auto record_residual =
      [&](const RootDemand& root, long long estimate) {
        if (represented_roots.count(root) != 0) return;
        auto [it, inserted] =
            residual_by_root.emplace(root, estimate);
        if (!inserted) it->second = std::max(it->second, estimate);
      };
  if (graph != nullptr) {
    for (size_t index = 0; index < graph->tasks.size(); ++index) {
      if (context != nullptr &&
          context->execution_view != nullptr &&
          index < context->execution_view->tasks.size() &&
          context->execution_view->tasks[index].state ==
              ExecutionTaskState::FULFILLED)
        continue;
      const auto& task = graph->tasks[index];
      int approach = INT_MAX / 4;
      for (const int robot_cell : physical.robots)
        approach = std::min(
            approach,
            lower_distance.dist(task.id.from, robot_cell));
      const long long estimate =
          (approach < INT_MAX / 4 ? approach : UNAVAILABLE) +
          task_service_ticks(task) +
          (index < critical_tail.size()
               ? critical_tail[index]
               : 0);
      for (const auto& root : task.roots)
        record_residual(root, estimate);
    }
    if (context != nullptr &&
        context->tau_guide != nullptr) {
      for (const int target : graph->paused_roots) {
        if (target < 0 ||
            target >= (int)physical.target_pos.size() ||
            target >= (int)context->tau_guide->size())
          continue;
        const int goal = (*context->tau_guide)[target];
        const RootDemand root{target, goal};
        int carrier = -1;
        for (size_t robot = 0;
             robot < physical.kappa.size(); ++robot)
          if (physical.kappa[robot] == target) {
            carrier = (int)robot;
            break;
          }
        const int shelf_distance =
            lower_distance.dist(
                goal, physical.target_pos[target]);
        long long estimate = UNAVAILABLE;
        if (shelf_distance < INT_MAX / 4) {
          if (carrier >= 0) {
            estimate = shelf_distance + 1;
          } else if (physical.target_pos[target] == goal) {
            estimate = 0;
          } else {
            int approach = INT_MAX / 4;
            for (const int robot_cell : physical.robots)
              approach = std::min(
                  approach,
                  lower_distance.dist(
                      physical.target_pos[target], robot_cell));
            if (approach < INT_MAX / 4)
              estimate =
                  (long long)approach + 1 +
                  shelf_distance + 1;
          }
        }
        record_residual(root, estimate);
      }
    }
  }
  std::vector<long long> residual_completion;
  long long residual_work = 0;
  for (const auto& [root, estimate] : residual_by_root) {
    (void)root;
    residual_completion.push_back(estimate);
    residual_work += estimate;
  }
  if (jobs.empty()) {
    const auto score =
        make_joint_transport_frame_score(
            {}, residual_completion, residual_work, {});
    out.predicted_all_targets_ticks =
        score.predicted_all_targets_ticks;
    out.predicted_work = score.predicted_work;
    return out;
  }

  std::vector<uint8_t> static_occupied(ins.grid.size(), 0);
  std::vector<uint8_t> carried_target(ins.n_targets(), 0);
  for (const int shelf : physical.kappa)
    if (shelf >= 0 && shelf < (int)ins.n_targets())
      carried_target[shelf] = 1;
  for (size_t target = 0; target < physical.target_pos.size(); ++target)
    if (!carried_target[target]) {
      const int cell = physical.target_pos[target];
      if (cell >= 0 && cell < ins.grid.size())
        static_occupied[cell] = 1;
    }
  for (const int cell : physical.anon_occ)
    if (cell >= 0 && cell < ins.grid.size())
      static_occupied[cell] = 1;
  for (size_t robot = 0; robot < physical.kappa.size() &&
                         robot < physical.robots.size();
       ++robot)
    if (physical.kappa[robot] != KAPPA_FREE &&
        (robot >= job_robot.size() || !job_robot[robot])) {
      const int cell = physical.robots[robot];
      if (cell >= 0 && cell < ins.grid.size())
        static_occupied[cell] = 1;
    }
  for (const auto& job : jobs)
    if (job.source >= 0 &&
        job.source < (int)static_occupied.size())
      static_occupied[job.source] = 0;

  struct ReservationTable {
    std::vector<std::vector<int>> vertex_owner;
    std::vector<std::set<std::pair<int, int>>> edges;
  };
  struct Frame {
    JointTransportGuidance guidance;
    JointTransportFrameScore score;
  };

  const auto search_job =
      [&](const Job& job,
          const ReservationTable& reservations) {
    TimedRouteHint hint;
    hint.endpoint = job.endpoint;
    if (job.source < 0 || job.source >= ins.grid.size() ||
        job.endpoint < 0 || job.endpoint >= ins.grid.size() ||
        job.topology_distance[job.source] >= INT_MAX / 4) {
      hint.status = RouteStatus::NO_ROUTE;
      return hint;
    }
    if (job.source == job.endpoint) {
      hint.status = RouteStatus::OK;
      hint.arrival_tick = job.start_tick;
      hint.cells.assign(horizon + 1, job.source);
      return hint;
    }
    if (job.start_tick >= horizon) {
      hint.status = RouteStatus::PREFIX;
      hint.cells.assign(horizon + 1, job.source);
      return hint;
    }
    if (expansions_per_job == 0) {
      hint.status = RouteStatus::BUDGET_EXHAUSTED;
      return hint;
    }

    const int cell_count = ins.grid.size();
    const int state_count = (horizon + 1) * cell_count;
    std::vector<int> parent(state_count, -2);
    std::deque<int> queue;
    const auto state_id =
        [&](int tick, int cell) {
          return tick * cell_count + cell;
        };
    const int start =
        state_id(job.start_tick, job.source);
    parent[start] = -1;
    queue.push_back(start);
    int goal_state = -1;
    bool budget_exhausted = false;
    int max_tick = job.start_tick;
    const auto can_hold_endpoint = [&](int from_tick) {
      for (int tick = from_tick; tick <= horizon; ++tick) {
        const int owner =
            reservations.vertex_owner[tick][job.endpoint];
        if (owner >= 0 && owner != job.robot) return false;
      }
      return true;
    };
    while (!queue.empty()) {
      if (hint.expansions >= expansions_per_job) {
        budget_exhausted = true;
        break;
      }
      const int state = queue.front();
      queue.pop_front();
      ++hint.expansions;
      const int tick = state / cell_count;
      const int cell = state % cell_count;
      max_tick = std::max(max_tick, tick);
      if (cell == job.endpoint &&
          can_hold_endpoint(tick)) {
        goal_state = state;
        break;
      }
      if (tick >= horizon) continue;

      int raw_neighbors[4];
      const int count =
          ins.grid.neighbors(cell, raw_neighbors);
      std::vector<int> next_cells(
          raw_neighbors, raw_neighbors + count);
      next_cells.push_back(cell);
      std::stable_sort(
          next_cells.begin(), next_cells.end(),
          [&](int a, int b) {
            const auto score = [&](int next) {
              return std::make_tuple(
                  job.topology_distance[next],
                  next == cell ? 1 : 0, next);
            };
            return score(a) < score(b);
          });
      for (const int next : next_cells) {
        const bool wait = next == cell;
        if (!wait && next != job.endpoint &&
            ins.can_store_shelf(next))
          continue;
        if (static_occupied[next]) continue;
        const int next_tick = tick + 1;
        const int owner =
            reservations.vertex_owner[next_tick][next];
        if (owner >= 0 && owner != job.robot) continue;
        if (!wait &&
            reservations.edges[next_tick].count(
                std::make_pair(next, cell)) != 0)
          continue;
        if (next == job.endpoint &&
            !can_hold_endpoint(next_tick))
          continue;
        const int next_state = state_id(next_tick, next);
        if (parent[next_state] != -2) continue;
        parent[next_state] = state;
        queue.push_back(next_state);
      }
    }

    int terminal = goal_state;
    if (terminal < 0 && max_tick >= job.start_tick) {
      int best_distance = INT_MAX;
      for (int state = max_tick * cell_count;
           state < (max_tick + 1) * cell_count; ++state) {
        if (parent[state] == -2) continue;
        const int cell = state % cell_count;
        const int distance = job.topology_distance[cell];
        if (distance < best_distance ||
            (distance == best_distance &&
             (terminal < 0 ||
              cell < terminal % cell_count))) {
          best_distance = distance;
          terminal = state;
        }
      }
    }
    if (terminal < 0) {
      hint.status = budget_exhausted
                        ? RouteStatus::BUDGET_EXHAUSTED
                        : RouteStatus::TEMPORARILY_BLOCKED;
      return hint;
    }
    std::vector<int> suffix;
    for (int state = terminal; state >= 0;
         state = parent[state])
      suffix.push_back(state % cell_count);
    std::reverse(suffix.begin(), suffix.end());
    hint.cells.assign(job.start_tick, job.source);
    hint.cells.insert(
        hint.cells.end(), suffix.begin(), suffix.end());
    if (goal_state >= 0) {
      hint.status = RouteStatus::OK;
      hint.arrival_tick = goal_state / cell_count;
    } else {
      hint.status = RouteStatus::PREFIX;
    }
    while ((int)hint.cells.size() < horizon + 1)
      hint.cells.push_back(hint.cells.back());
    return hint;
  };

  std::vector<std::vector<int>> orders;
  const auto add_order = [&](std::vector<int> order) {
    if ((int)orders.size() >= frame_budget) return;
    if (std::find(orders.begin(), orders.end(), order) ==
        orders.end())
      orders.push_back(std::move(order));
  };
  std::vector<int> base(jobs.size());
  std::iota(base.begin(), base.end(), 0);
  add_order(base);
  if (jobs.size() > 1) {
    auto reverse = base;
    std::reverse(reverse.begin(), reverse.end());
    add_order(std::move(reverse));
    for (size_t shift = 1;
         shift < jobs.size() &&
         (int)orders.size() < frame_budget;
         ++shift) {
      auto rotated = base;
      std::rotate(
          rotated.begin(), rotated.begin() + shift,
          rotated.end());
      add_order(std::move(rotated));
    }
    for (size_t index = 0;
         index + 1 < jobs.size() &&
         (int)orders.size() < frame_budget;
         ++index) {
      auto swapped = base;
      std::swap(swapped[index], swapped[index + 1]);
      add_order(std::move(swapped));
    }
  }

  std::optional<Frame> best;
  int total_expansions = 0;
  int frames_evaluated = 0;
  for (const auto& order : orders) {
    Frame frame;
    frame.guidance.by_robot.resize(ins.n_robots());
    ReservationTable reservations;
    reservations.vertex_owner.assign(
        horizon + 1,
        std::vector<int>(ins.grid.size(), -1));
    reservations.edges.resize(horizon + 1);
    for (const auto& job : jobs) {
      if (job.source < 0 ||
          job.source >= ins.grid.size())
        continue;
      const int hold_until =
          job.assigned
              ? std::min(horizon, job.start_tick)
              : 0;
      for (int tick = 0; tick <= hold_until; ++tick)
        reservations.vertex_owner[tick][job.source] =
            job.robot;
    }

    std::vector<long long> planned_completion;
    long long predicted_work = residual_work;
    for (const int job_index : order) {
      const auto& job = jobs[job_index];
      TimedRouteHint hint =
          search_job(job, reservations);
      frame.guidance.expansions += hint.expansions;
      if (hint.cells.empty())
        hint.cells.assign(horizon + 1, job.source);
      bool reservation_conflict = false;
      for (int tick = 0; tick <= horizon; ++tick) {
        const int cell = hint.cells[tick];
        const int owner =
            reservations.vertex_owner[tick][cell];
        if (owner >= 0 && owner != job.robot) {
          reservation_conflict = true;
          continue;
        }
        reservations.vertex_owner[tick][cell] =
            job.robot;
        if (tick > 0 &&
            hint.cells[tick - 1] != cell)
          reservations.edges[tick].insert(
              std::make_pair(
                  hint.cells[tick - 1], cell));
      }
      int moves = 0;
      for (size_t tick = 1; tick < hint.cells.size(); ++tick)
        moves += hint.cells[tick] != hint.cells[tick - 1];
      const int remaining =
          job.topology_distance[hint.cells.back()];
      long long finish = 0;
      if (!reservation_conflict &&
          hint.status == RouteStatus::OK) {
        finish =
            hint.arrival_tick + 1 + job.tail_ticks;
      } else if (!reservation_conflict &&
                 hint.status == RouteStatus::PREFIX &&
                 remaining < INT_MAX / 4) {
        finish =
            (long long)std::max(horizon, job.start_tick) +
            remaining + 1 +
            job.tail_ticks;
      } else {
        finish =
            UNAVAILABLE + horizon + job.tail_ticks;
      }
      planned_completion.push_back(finish);
      predicted_work +=
          job.approach_ticks + (job.assigned ? 1 : 0) +
          moves + 1 +
          (remaining < INT_MAX / 4
               ? remaining
               : UNAVAILABLE);
      frame.guidance.by_robot[job.robot] = std::move(hint);
    }
    frame.score =
        make_joint_transport_frame_score(
            planned_completion, residual_completion,
            predicted_work, order);
    frame.guidance.predicted_all_targets_ticks =
        frame.score.predicted_all_targets_ticks;
    frame.guidance.predicted_work =
        frame.score.predicted_work;
    total_expansions += frame.guidance.expansions;
    ++frames_evaluated;
    if (!best.has_value() ||
        better_joint_transport_frame(
            frame.score, best->score))
      best = std::move(frame);
  }
  if (!best.has_value()) return out;
  out = std::move(best->guidance);
  out.expansions = total_expansions;
  out.frames_evaluated = frames_evaluated;
  return out;
}

}  // namespace carrier_detail
