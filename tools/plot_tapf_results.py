#!/usr/bin/env python3
"""Plot TAPF experiment cost comparisons."""

from __future__ import annotations

import argparse
import csv
import heapq
import math
from collections import defaultdict, deque
from pathlib import Path
from typing import Any, Dict, List, Tuple

import matplotlib.pyplot as plt
import yaml


# ITA-CBS YAML stores coordinates as [row, col]. Keep the same convention here.
Coord = Tuple[int, int]
INF = 10**9


def load_yaml(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_map(map_path: Path) -> Tuple[int, int, set[Coord]]:
    lines = map_path.read_text(encoding="utf-8").splitlines()
    height = int(next(line.split()[1] for line in lines if line.startswith("height")))
    width = int(next(line.split()[1] for line in lines if line.startswith("width")))
    map_start = lines.index("map") + 1
    obstacles: set[Coord] = set()
    for row, line in enumerate(lines[map_start : map_start + height]):
        for col, char in enumerate(line):
            if char in ("@", "T"):
                obstacles.add((row, col))
    return width, height, obstacles


def bfs_distances(width: int, height: int, obstacles: set[Coord], goal: Coord) -> Dict[Coord, int]:
    dist = {goal: 0}
    q = deque([goal])
    while q:
        row, col = q.popleft()
        for nr, nc in ((row + 1, col), (row - 1, col), (row, col + 1), (row, col - 1)):
            if nr < 0 or nc < 0 or nr >= height or nc >= width:
                continue
            if (nr, nc) in obstacles or (nr, nc) in dist:
                continue
            dist[(nr, nc)] = dist[(row, col)] + 1
            q.append((nr, nc))
    return dist


def hungarian_min(cost: List[List[int]]) -> int:
    n = len(cost)
    u = [0] * (n + 1)
    v = [0] * (n + 1)
    p = [0] * (n + 1)
    way = [0] * (n + 1)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [INF] * (n + 1)
        used = [False] * (n + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = INF
            j1 = 0
            for j in range(1, n + 1):
                if used[j]:
                    continue
                cur = cost[i0 - 1][j - 1] - u[i0] - v[j]
                if cur < minv[j]:
                    minv[j] = cur
                    way[j] = j0
                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j
            for j in range(n + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    return -v[0]


def optimistic_assignment_lb(instance_file: Path, map_dir: Path) -> int:
    data = load_yaml(instance_file)
    map_path = map_dir / data["map"]
    width, height, obstacles = load_map(map_path)

    unique_goals: List[Coord] = []
    goal_to_idx: Dict[Coord, int] = {}
    starts: List[Coord] = []
    allowed: List[List[Coord]] = []
    for agent in data["agents"]:
        starts.append(tuple(agent["start"]))
        if "potentialGoals" in agent:
            goals = [tuple(goal) for goal in agent["potentialGoals"]]
        else:
            goals = [tuple(agent["goal"])]
        allowed.append(goals)
        for goal in goals:
            if goal not in goal_to_idx:
                goal_to_idx[goal] = len(unique_goals)
                unique_goals.append(goal)

    dists = [bfs_distances(width, height, obstacles, goal) for goal in unique_goals]
    cost = [[INF for _ in unique_goals] for _ in starts]
    for i, start in enumerate(starts):
        for goal in allowed[i]:
            j = goal_to_idx[goal]
            cost[i][j] = dists[j].get(start, INF)
    return hungarian_min(cost)


def read_results(csv_path: Path) -> Dict[Tuple[int, int], Dict[str, Dict[str, Any]]]:
    grouped: Dict[Tuple[int, int], Dict[str, Dict[str, Any]]] = defaultdict(dict)
    with csv_path.open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            key = (int(row["case_agents"]), int(row["case_test"]))
            grouped[key][row["solver"]] = row
    return grouped


def as_float(row: Dict[str, Any], key: str) -> float:
    value = row.get(key, "")
    return float(value) if value != "" else math.nan


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=Path("build/results/tapf_full_1s_timeout1.csv"))
    parser.add_argument("--map-dir", type=Path, default=Path("third_party/ITA-CBS2/map_file"))
    parser.add_argument("--out-dir", type=Path, default=Path("build/plots/tapf_results"))
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    results = read_results(args.csv)
    ratio_rows: List[Dict[str, Any]] = []

    paired_x = []
    paired_y = []
    paired_agents = []
    for (agents, test), solvers in sorted(results.items()):
        lacam = solvers.get("lacam_tapf")
        itacbs = solvers.get("itacbs")
        fixture = Path(lacam["instance_file"] if lacam else itacbs["instance_file"])
        lb = optimistic_assignment_lb(fixture, args.map_dir)

        for solver, row in solvers.items():
            solved = row.get("solved") == "1"
            soc = as_float(row, "soc") if solved else math.nan
            ratio_rows.append(
                {
                    "case_agents": agents,
                    "case_test": test,
                    "solver": solver,
                    "solved": int(solved),
                    "soc": soc,
                    "optimistic_assignment_lb": lb,
                    "soc_over_lb": soc / lb if solved and lb > 0 else math.nan,
                }
            )

        if lacam and itacbs and lacam.get("solved") == "1" and itacbs.get("solved") == "1":
            paired_x.append(as_float(itacbs, "soc"))
            paired_y.append(as_float(lacam, "soc"))
            paired_agents.append(agents)

    ratio_csv = args.out_dir / "tapf_cost_ratios.csv"
    with ratio_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "case_agents",
                "case_test",
                "solver",
                "solved",
                "soc",
                "optimistic_assignment_lb",
                "soc_over_lb",
            ],
        )
        writer.writeheader()
        writer.writerows(ratio_rows)

    plt.figure(figsize=(7, 6))
    scatter = plt.scatter(paired_x, paired_y, c=paired_agents, cmap="viridis", s=48, alpha=0.85)
    if paired_x and paired_y:
        lo = min(min(paired_x), min(paired_y))
        hi = max(max(paired_x), max(paired_y))
        plt.plot([lo, hi], [lo, hi], color="black", linestyle="--", linewidth=1, label="equal cost")
    plt.xlabel("ITA-CBS cost")
    plt.ylabel("LaCAM-TAPF cost")
    plt.title("Cost on Same Solved Testcases")
    plt.colorbar(scatter, label="agents")
    plt.legend()
    plt.tight_layout()
    scatter_path = args.out_dir / "cost_lacam_vs_itacbs.png"
    plt.savefig(scatter_path, dpi=180)
    plt.close()

    by_solver_agent: Dict[Tuple[str, int], List[float]] = defaultdict(list)
    for row in ratio_rows:
        if row["solved"] and not math.isnan(row["soc_over_lb"]):
            by_solver_agent[(row["solver"], row["case_agents"])].append(row["soc_over_lb"])

    plt.figure(figsize=(8, 5.5))
    for solver, marker in [("lacam_tapf", "o"), ("itacbs", "s")]:
        xs = []
        ys = []
        yerr = []
        for agents in sorted({key[1] for key in by_solver_agent if key[0] == solver}):
            vals = by_solver_agent[(solver, agents)]
            mean_value = sum(vals) / len(vals)
            xs.append(agents)
            ys.append(mean_value)
            yerr.append(
                (0, 0)
                if len(vals) == 1
                else (mean_value - min(vals), max(vals) - mean_value)
            )
        lower_err = [err[0] for err in yerr]
        upper_err = [err[1] for err in yerr]
        plt.errorbar(
            xs,
            ys,
            yerr=[lower_err, upper_err],
            marker=marker,
            capsize=3,
            label=solver,
        )
    plt.xlabel("number of agents")
    plt.ylabel("cost / optimistic initial assignment lower bound")
    plt.title("Cost Ratio vs Agent Count")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    ratio_path = args.out_dir / "cost_over_lb_by_agents.png"
    plt.savefig(ratio_path, dpi=180)
    plt.close()

    print(scatter_path)
    print(ratio_path)
    print(ratio_csv)
    print(f"paired solved cases: {len(paired_x)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
