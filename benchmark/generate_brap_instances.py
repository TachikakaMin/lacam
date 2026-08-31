#!/usr/bin/env python3
"""Generate test cases following the BRaP protocol (arXiv:2509.01022,
AAAI-26, Table 2 + Figure 6), adapted to the two-deck carrier model.

Paper layout protocol (replicated exactly):
  - grid sizes 4x10, 6x10, 8x10, 10x10, 20x20, 40x40, 80x80;
  - obstacle: square of side floor(L/5) (L = longer grid dimension) in the
    bottom-right corner;
  - EVERY non-obstacle cell holds a block except `n_empty` empty cells
    (>= 75% fill: "extremely dense");
  - assigned blocks: <= 12.5% of |V|; for Goal B additionally capped at
    2x grid height (Table 2 footnote);
  - empty vertices: 1 .. 25% of |V|;
  - goal types: B = boundary vertices, R1 = random cells (1x assigned);
  - concrete (assigned, empty) combos calibrated to the paper's Figure 6
    examples (e.g. 10x10/13/{3,8}, 20x20/40/10, 40x40/160/40,
    80x80/{128,800}x{160,1600}).

Honest adaptations to OUR model (documented in benchmark/README.md):
  - BRaP blocks move themselves; the carrier model needs robots.  We add
    n_robots = max(2, n_assigned // 8) (cap 32) at seeded random cells
    (robots may start under shelves - legal in the two-deck model, I3).
  - BRaP goals are an interchangeable SET; our v1 semantics needs one
    FIXED goal per target.  We fix the assignment by deterministic
    greedy nearest matching (mirrors BR-PIBT's "closest unallocated
    goal" allocation).  This is conservative for us.
  - We allow following (design 3.4a); BRaP forbids it.  Completed
    targets stay liftable (D2); BRaP pins them as obstacles.

Deterministic: everything derives from the per-instance seed.
Instances land in instances_brap/<group>/ .  PROTECTED once generated:
regeneration must reproduce byte-identical files (fixed seeds).
"""
import random
from pathlib import Path

OUT = Path(__file__).parent / "instances_brap"
SEEDS = [0, 1]

# (h, w, [(n_assigned_R1, n_empty), ...])  calibrated to Figure 6;
# Goal B re-caps n_assigned at min(12.5%|V|, 2h) per Table 2 footnote.
GRIDS = [
    (4, 10, [(5, 1), (5, 10)]),
    (6, 10, [(6, 1), (6, 15)]),
    (8, 10, [(10, 2), (10, 20)]),
    (10, 10, [(13, 3), (13, 8), (1, 1)]),      # (1,1) = fig 6h deep-bury
    (20, 20, [(40, 10), (40, 100)]),
    (40, 40, [(160, 40), (160, 400)]),
    (80, 80, [(128, 160), (128, 1600), (800, 160), (800, 1600)]),
]


def obstacle_cells(h, w):
    side = max(1, max(h, w) // 5)
    side = min(side, h, w)
    return {(r, c)
            for r in range(h - side, h)
            for c in range(w - side, w)}


def boundary_cells(h, w, obst):
    cells = []
    for r in range(h):
        for c in range(w):
            if (r == 0 or r == h - 1 or c == 0 or c == w - 1) \
                    and (r, c) not in obst:
                cells.append((r, c))
    return cells


def greedy_nearest_match(starts, goals):
    """deterministic greedy: repeatedly take the globally closest
    (start, goal) pair (mirrors BR-PIBT closest-unallocated-goal)."""
    pairs = []
    starts = list(starts)
    goals = list(goals)
    while starts:
        best = None
        for i, s in enumerate(starts):
            for j, g in enumerate(goals):
                d = abs(s[0] - g[0]) + abs(s[1] - g[1])
                key = (d, s, g)
                if best is None or key < best[0]:
                    best = (key, i, j)
        _, i, j = best
        pairs.append((starts.pop(i), goals.pop(j)))
    return pairs


def gen_instance(h, w, n_assigned, n_empty, goal_type, seed):
    rng = random.Random((h * 1000003 + w * 1009 + n_assigned * 97 +
                         n_empty * 13 + (7 if goal_type == "B" else 11)) *
                        1000 + seed)
    obst = obstacle_cells(h, w)
    free = [(r, c) for r in range(h) for c in range(w) if (r, c) not in obst]
    n_cells = h * w

    if goal_type == "B":
        n_assigned = min(n_assigned, n_cells // 8, 2 * h)
        goal_pool = boundary_cells(h, w, obst)
        if n_assigned > len(goal_pool):
            n_assigned = len(goal_pool)
    else:
        n_assigned = min(n_assigned, n_cells // 8)

    # empties, then every remaining free cell is a shelf (dense fill)
    empties = set(rng.sample(free, n_empty))
    shelves = [cell for cell in free if cell not in empties]
    assigned = rng.sample(shelves, n_assigned)

    if goal_type == "B":
        goals_set = rng.sample(goal_pool, n_assigned)
    else:
        goals_set = rng.sample(free, n_assigned)  # R1: random cells, 1x
    pairs = greedy_nearest_match(assigned, goals_set)

    n_robots = max(2, min(32, n_assigned // 8))
    robots = rng.sample(free, n_robots)  # may start under shelves (I3)

    rows = ["".join("@" if (r, c) in obst else "." for c in range(w))
            for r in range(h)]
    name = f"brap_h{h}w{w}_a{n_assigned}_e{n_empty}_{goal_type}_seed{seed}"
    y = [f"name: {name}", "map: |"]
    y += [f"  {row}" for row in rows]
    y.append("robots:")
    y += [f"  - [{r}, {c}]" for r, c in robots]
    y.append("shelves:")
    y += [f"  - [{r}, {c}]" for r, c in shelves]
    y.append("targets:")
    for k, (s, g) in enumerate(pairs):
        y.append(f"  - id: b{k}")
        y.append(f"    start: [{s[0]}, {s[1]}]")
        y.append(f"    goal: [{g[0]}, {g[1]}]")
    return name, "\n".join(y) + "\n"


def main():
    n = 0
    for h, w, combos in GRIDS:
        for n_assigned, n_empty in combos:
            for goal_type in ("B", "R1"):
                group = OUT / f"g{h}x{w}"
                group.mkdir(parents=True, exist_ok=True)
                for seed in SEEDS:
                    name, text = gen_instance(h, w, n_assigned, n_empty,
                                              goal_type, seed)
                    (group / f"{name}.yaml").write_text(text)
                    n += 1
    print(f"generated {n} BRaP-protocol instances under {OUT}/")


if __name__ == "__main__":
    main()
