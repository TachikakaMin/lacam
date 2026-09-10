// Internal helpers shared by the tapf_planner_*.cpp translation units
// (split of the original tapf_planner.cpp).  NOT part of the public API.
#pragma once

#include "../include/tapf_planner.hpp"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <map>
#include <queue>
#include <set>
#include <unordered_set>
#include "../include/search_kernel.hpp"
#include "carrier_guidance.hpp"

namespace tapf_detail
{
  inline double focal_score(const TAPFNode* node, TAPFFocalTieBreak tie_break)
  {
    switch (tie_break) {
      case TAPFFocalTieBreak::ANTI_WAIT:
        return 8 * node->non_goal_waits + 4 * node->reversals +
               2 * node->distance_increases + node->settled_pushes;
      case TAPFFocalTieBreak::ANTI_ZIGZAG:
        return 8 * node->reversals + 4 * node->distance_increases +
               2 * node->settled_pushes + node->non_goal_waits;
      case TAPFFocalTieBreak::ANTI_PUSH:
        return 8 * node->settled_pushes + 4 * node->reversals +
               2 * node->distance_increases + node->non_goal_waits;
      case TAPFFocalTieBreak::ANTI_ALL:
        return 10 * node->settled_pushes + 6 * node->reversals +
               3 * node->non_goal_waits + 2 * node->distance_increases;
      case TAPFFocalTieBreak::H:
      default:
        return node->h.work_value();
    }
  }

  inline bool focal_better(const TAPFNode* a, const TAPFNode* b,
                    TAPFFocalTieBreak tie_break)
  {
    if (a->h != b->h) return a->h < b->h;
    const auto a_score = focal_score(a, tie_break);
    const auto b_score = focal_score(b, tie_break);
    if (a_score != b_score) return a_score < b_score;
    if (a->f != b->f) return a->f < b->f;
    if (a->g != b->g) return a->g > b->g;
    return a->depth < b->depth;
  }

  // CLOSED key = (Config, ShelfState) (design 6.1, mapping M2/M14): the
  // shelf hash is splitmix64-derived per (role, id, cell) and XORs to 0
  // for the empty layer, so shelf-free instances keep the exact original
  // ConfigHasher value and duplicate semantics.
  inline uint64_t splitmix64(uint64_t x)
  {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
  }

  inline uint64_t shelf_layer_hash(const ShelfState& S)
  {
    uint64_t h = 0;
    for (size_t b = 0; b < S.target_pos.size(); ++b)
      h ^= splitmix64((1ULL << 40) ^ (b << 20) ^ (uint64_t)S.target_pos[b]);
    for (const int c : S.anon_occ)
      h ^= splitmix64((2ULL << 40) ^ (uint64_t)c);
    for (size_t i = 0; i < S.kappa.size(); ++i)
      h ^= splitmix64((3ULL << 40) ^ (i << 20) ^
                      (uint64_t)(S.kappa[i] + 2));
    return h;
  }

  inline PhysConfig physical_state_of(const Config& C, const ShelfState& S)
  {
    PhysConfig out;
    out.robots.reserve(C.size());
    for (const auto* vertex : C) out.robots.push_back(vertex->index);
    out.target_pos = S.target_pos;
    out.anon_occ = S.anon_occ;
    out.kappa = S.kappa;
    return out;
  }

  inline Config config_of_physical(const TAPFInstance& ins,
                            const PhysConfig& physical)
  {
    Config out;
    out.reserve(physical.robots.size());
    for (const int cell : physical.robots) out.push_back(ins.G.U[cell]);
    return out;
  }

  inline ShelfState shelf_of_physical(const PhysConfig& physical)
  {
    ShelfState out;
    out.target_pos = physical.target_pos;
    out.anon_occ = physical.anon_occ;
    out.kappa = physical.kappa;
    return out;
  }

  struct SearchKey {
    Config C;
    ShelfState S;
    int64_t commitment_phase = 0;
    bool operator==(const SearchKey& o) const
    {
      return C == o.C && S == o.S &&
             commitment_phase == o.commitment_phase;
    }
  };

  struct SearchKeyHasher {
    size_t operator()(const SearchKey& k) const
    {
      size_t h =
          (size_t)ConfigHasher()(k.C) ^
          (size_t)shelf_layer_hash(k.S);
      if (k.commitment_phase != 0)
        h ^= (size_t)splitmix64(
            static_cast<uint64_t>(k.commitment_phase));
      return h;
    }
  };

  // guidance infrastructure lives in carrier_guidance.hpp (shared with
  // the carrier adapters in dd_planner.cpp)
  using namespace carrier_detail;

  constexpr int MACRO_CAP = 64;
  constexpr int MACRO_TARGET_LIMIT = 64;

  inline double focal_numeric_cost(const PlanCost& cost, TAPFObjective objective)
  {
    if (!cost.is_bounded()) return -1;
    return objective == TAPFObjective::MAKESPAN_THEN_WORK
               ? static_cast<double>(cost.ticks)
               : cost.work_value();
  }

  inline void add_scaled_work(int64_t& total, int64_t amount)
  {
    if (amount < 0 ||
        total > std::numeric_limits<int64_t>::max() - amount)
      throw std::overflow_error("fixed-point search work overflow");
    total += amount;
  }

  inline std::optional<PlanCost> reference_joint_cost(
      const SolverWeights& weights, const PhysConfig& state,
      const std::vector<Op>& ops, TAPFObjective objective)
  {
    if (ops.size() != state.robots.size() ||
        state.kappa.size() != state.robots.size())
      return std::nullopt;
    int64_t work = 0;
    for (size_t i = 0; i < ops.size(); ++i) {
      if (ops[i].kind == Op::MOVE) {
        add_scaled_work(
            work, state.kappa[i] == KAPPA_FREE
                      ? weights.beta_scaled
                      : weights.alpha_scaled);
        if (state.kappa[i] == KAPPA_ANON)
          add_scaled_work(work, weights.delta_scaled);
      } else if (ops[i].kind == Op::LIFT ||
                 ops[i].kind == Op::DROP) {
        add_scaled_work(work, weights.gamma_scaled);
      }
    }
    return PlanCost::from_scaled(
        objective == TAPFObjective::MAKESPAN_THEN_WORK ? 1 : 0,
        work);
  }
}  // namespace tapf_detail

// Complete type for TAPFPlanner::carrier (unique_ptr member).
using namespace carrier_detail;

struct TAPFPlanner::CarrierEngine {
  std::shared_ptr<TAPFCarrierPersistentState> persistent;
  // upper-deck wall distance (design_final 6.2/D21): ONE shared
  // dest-keyed cache — the field depends only on (walls, dest), so
  // per-target copies were redundant and would duplicate massively
  // under shared goal pools.
  DDDistCache& upper_wall;
  StorageTransferTopology& storage_topology;
  LowerDist& lower;
  UpperEpochCache& task_br_cache;
  PhysConfig phys;  // scratch physical view of the node in processing

  explicit CarrierEngine(
      const DDInstance& dd,
      std::shared_ptr<TAPFCarrierPersistentState> shared = nullptr)
      : persistent(
            shared != nullptr
                ? std::move(shared)
                : std::make_shared<TAPFCarrierPersistentState>(dd)),
        upper_wall(persistent->upper_wall),
        storage_topology(persistent->storage_topology),
        lower(persistent->lower),
        task_br_cache(persistent->task_br_cache)
  {
    if (!persistent->compatible_with(dd))
      throw std::invalid_argument(
          "Carrier persistent state schema mismatch");
  }

  // physical view of a node (oracle coordinates)
  const PhysConfig& phys_view(const TAPFNode* nd)
  {
    phys.robots.resize(nd->C.size());
    for (size_t i = 0; i < nd->C.size(); ++i)
      phys.robots[i] = nd->C[i]->index;
    phys.target_pos = nd->shelf.target_pos;
    phys.anon_occ = nd->shelf.anon_occ;
    phys.kappa = nd->shelf.kappa;
    return phys;
  }
};
