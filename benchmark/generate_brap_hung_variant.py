#!/usr/bin/env python3
"""Hungarian-pairing variant of the BRaP suite (ablation; see README).
Same RNG streams as generate_brap_instances.py — only the static
start->goal pairing differs: optimal min-total-Manhattan assignment
(pure-python O(n^3) potentials Hungarian) instead of greedy nearest.
Outputs to instances_brap_hung/ (the protected instances_brap/ suite is
untouched)."""
import importlib.util, random
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    "gbi", Path(__file__).parent / "generate_brap_instances.py")
gbi = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gbi)


def hungarian(cost):
    n, m = len(cost), len(cost[0])
    INF = float('inf')
    u = [0]*(n+1); v = [0]*(m+1); p = [0]*(m+1); way = [0]*(m+1)
    for i in range(1, n+1):
        p[0] = i; j0 = 0
        minv = [INF]*(m+1); used = [False]*(m+1)
        while True:
            used[j0] = True
            i0 = p[j0]; delta = INF; j1 = 0
            row = cost[i0-1]
            for j in range(1, m+1):
                if used[j]:
                    continue
                cur = row[j-1] - u[i0] - v[j]
                if cur < minv[j]:
                    minv[j] = cur; way[j] = j0
                if minv[j] < delta:
                    delta = minv[j]; j1 = j
            for j in range(m+1):
                if used[j]:
                    u[p[j]] += delta; v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:
            j1 = way[j0]; p[j0] = p[j1]; j0 = j1
    r2c = [-1]*n
    for j in range(1, m+1):
        if p[j] > 0:
            r2c[p[j]-1] = j-1
    return r2c


def main():
    out = Path(__file__).parent / "instances_brap_hung"
    n_files = 0
    for h, w, combos in gbi.GRIDS:
        for n_assigned, n_empty in combos:
            for goal_type in ("B", "R1"):
                group = out / f"g{h}x{w}"
                group.mkdir(parents=True, exist_ok=True)
                for seed in gbi.SEEDS:
                    rng = random.Random(
                        (h*1000003 + w*1009 + n_assigned*97 + n_empty*13 +
                         (7 if goal_type == "B" else 11)) * 1000 + seed)
                    obst = gbi.obstacle_cells(h, w)
                    free = [(r, c) for r in range(h) for c in range(w)
                            if (r, c) not in obst]
                    n_cells = h * w
                    if goal_type == "B":
                        na = min(n_assigned, n_cells // 8, 2*h)
                        pool = gbi.boundary_cells(h, w, obst)
                        na = min(na, len(pool))
                    else:
                        na = min(n_assigned, n_cells // 8)
                    empties = set(rng.sample(free, n_empty))
                    shelves = [c for c in free if c not in empties]
                    assigned = rng.sample(shelves, na)
                    goals = rng.sample(pool if goal_type == "B" else free, na)
                    cost = [[abs(s[0]-g[0]) + abs(s[1]-g[1]) for g in goals]
                            for s in assigned]
                    r2c = hungarian(cost)
                    pairs = [(assigned[i], goals[r2c[i]]) for i in range(na)]
                    n_rob = max(2, min(32, na // 8))
                    robots = rng.sample(free, n_rob)
                    rows = ["".join("@" if (r, c) in obst else "."
                                    for c in range(w)) for r in range(h)]
                    name = (f"braph_h{h}w{w}_a{na}_e{n_empty}_"
                            f"{goal_type}_seed{seed}")
                    y = [f"name: {name}", "map: |"]
                    y += [f"  {r}" for r in rows]
                    y.append("robots:")
                    y += [f"  - [{r}, {c}]" for r, c in robots]
                    y.append("shelves:")
                    y += [f"  - [{r}, {c}]" for r, c in shelves]
                    y.append("targets:")
                    for k, (s, g) in enumerate(pairs):
                        y += [f"  - id: b{k}",
                              f"    start: [{s[0]}, {s[1]}]",
                              f"    goal: [{g[0]}, {g[1]}]"]
                    (group / f"{name}.yaml").write_text("\n".join(y) + "\n")
                    n_files += 1
    print(f"generated {n_files} Hungarian-paired instances under {out}/")


if __name__ == "__main__":
    main()
