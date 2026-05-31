#!/usr/bin/env python3
from __future__ import annotations

import argparse
import collections
import re
from pathlib import Path

import yaml


def agent_index(name: str) -> int:
    match = re.search(r"\d+", name)
    return int(match.group()) if match else 0


def read_map(path: Path) -> list[str]:
    grid = []
    in_map = False
    for line in path.read_text().splitlines():
        if line.strip() == "map":
            in_map = True
            continue
        if in_map:
            grid.append(line.rstrip())
    if not grid:
        raise ValueError(f"no MovingAI map section found in {path}")
    return grid


def free(grid: list[str], pos: tuple[int, int]) -> bool:
    row, col = pos
    return 0 <= row < len(grid) and 0 <= col < len(grid[0]) and grid[row][col] != "@"


def neighbors(grid: list[str], pos: tuple[int, int]) -> list[tuple[int, int]]:
    row, col = pos
    # Matches graph.cpp insertion order: left, right, y + 1, y - 1.
    out = [(row, col - 1), (row, col + 1), (row + 1, col), (row - 1, col)]
    return [p for p in out if free(grid, p)]


def bfs(grid: list[str], source: tuple[int, int]) -> dict[tuple[int, int], int]:
    queue = collections.deque([source])
    dist = {source: 0}
    while queue:
        pos = queue.popleft()
        for nxt in neighbors(grid, pos):
            if nxt in dist:
                continue
            dist[nxt] = dist[pos] + 1
            queue.append(nxt)
    return dist


def expand_schedule(schedule: dict, makespan: int) -> tuple[list[list[tuple[int, int]]], list[tuple[int, int]]]:
    paths = []
    goals = []
    for name in sorted(schedule["schedule"], key=agent_index):
        assignment = schedule["assignments"][name]
        goals.append((int(assignment["x"]), int(assignment["y"])))
        entries = [(int(step["t"]), (int(step["x"]), int(step["y"]))) for step in schedule["schedule"][name]]
        dense = [None] * (makespan + 1)
        for idx, (t, pos) in enumerate(entries):
            next_t = entries[idx + 1][0] if idx + 1 < len(entries) else makespan + 1
            for tt in range(t, min(next_t, makespan + 1)):
                dense[tt] = pos
        if any(pos is None for pos in dense):
            raise ValueError(f"incomplete path for {name}")
        paths.append(dense)
    return paths, goals


def agent_stats(path: list[tuple[int, int]], goal: tuple[int, int]) -> dict:
    makespan = len(path) - 1
    first = next((t for t, pos in enumerate(path) if pos == goal), None)
    soc = makespan + 1
    while soc > 0 and path[soc - 1] == goal:
        soc -= 1
    sol = sum(1 for t in range(1, makespan + 1) if path[t - 1] != goal or path[t] != goal)
    moves = sum(1 for t in range(1, makespan + 1) if path[t] != path[t - 1])
    compressed = []
    for pos in path:
        if not compressed or compressed[-1] != pos:
            compressed.append(pos)
    aba = sum(
        1
        for idx in range(2, len(compressed))
        if compressed[idx] == compressed[idx - 2] and compressed[idx - 1] != compressed[idx]
    )
    hidden_wait = sum(1 for t in range(first or 0, soc) if path[t] == goal) if first is not None else 0
    off_after = sum(1 for t in range(first or 0, soc) if path[t] != goal) if first is not None else 0
    return {
        "first": first,
        "soc": soc,
        "sol": sol,
        "moves": moves,
        "aba": aba,
        "hidden_wait": hidden_wait,
        "off_after": off_after,
        "distinct": len(set(path)),
    }


def reconstruct_priorities(paths: list[list[tuple[int, int]]], dists: list[dict[tuple[int, int], int]]) -> list[list[float]]:
    agent_count = len(paths)
    makespan = len(paths[0]) - 1
    priorities = [[0.0] * agent_count for _ in range(makespan + 1)]
    for i in range(agent_count):
        priorities[0][i] = dists[i][paths[i][0]] / agent_count
    for t in range(1, makespan + 1):
        for i in range(agent_count):
            if dists[i][paths[i][t]] != 0:
                priorities[t][i] = priorities[t - 1][i] + 1
            else:
                priorities[t][i] = priorities[t - 1][i] - int(priorities[t - 1][i])
    return priorities


def direct_pushes(paths: list[list[tuple[int, int]]]) -> tuple[collections.Counter, dict]:
    makespan = len(paths[0]) - 1
    agent_count = len(paths)
    counts = collections.Counter()
    times = collections.defaultdict(list)
    for t in range(1, makespan + 1):
        prev_at = {paths[i][t - 1]: i for i in range(agent_count)}
        for pusher in range(agent_count):
            if paths[pusher][t] == paths[pusher][t - 1]:
                continue
            pushed = prev_at.get(paths[pusher][t])
            if pushed is None or pushed == pusher or paths[pushed][t] == paths[pushed][t - 1]:
                continue
            counts[(pusher, pushed)] += 1
            if len(times[(pusher, pushed)]) < 20:
                times[(pusher, pushed)].append(t)
    return counts, times


def print_top_agents(stats: list[dict], title: str, key: str, count: int) -> None:
    print(f"\n{title}")
    for i, row in sorted(enumerate(stats), key=lambda item: item[1][key], reverse=True)[:count]:
        print(
            f"agent{i:02d} soc={row['soc']:4} sol={row['sol']:4} first={row['first']:3} "
            f"moves={row['moves']:3} ABA={row['aba']:3} hidden={row['hidden_wait']:4} "
            f"off={row['off_after']:3} distinct={row['distinct']:3}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("map", type=Path)
    parser.add_argument("schedule", type=Path)
    parser.add_argument("--agent", type=int, default=13)
    parser.add_argument("--events", type=int, nargs="*", default=[138, 151, 156, 164, 174, 202, 253, 298, 406, 426, 502, 550, 596, 601, 664])
    args = parser.parse_args()

    grid = read_map(args.map)
    schedule = yaml.safe_load(args.schedule.read_text())
    makespan = int(schedule["statistics"]["makespan"])
    paths, goals = expand_schedule(schedule, makespan)
    dists = [bfs(grid, goal) for goal in goals]
    priorities = reconstruct_priorities(paths, dists)
    stats = [agent_stats(path, goals[i]) for i, path in enumerate(paths)]
    pushes, push_times = direct_pushes(paths)

    print(f"makespan={makespan}")
    print(f"soc={sum(row['soc'] for row in stats)}")
    print(f"sum_of_loss={sum(row['sol'] for row in stats)}")
    print(f"moves={sum(row['moves'] for row in stats)}")
    print(f"aba={sum(row['aba'] for row in stats)}")
    print(f"hidden_wait={sum(row['hidden_wait'] for row in stats)}")
    print(f"off_after_goal={sum(row['off_after'] for row in stats)}")

    print_top_agents(stats, "Top SOC contributors", "soc", 12)
    print_top_agents(stats, "Top zigzag contributors", "aba", 12)
    print_top_agents(stats, "Top wandering/distinct-cell contributors", "distinct", 12)

    print("\nTop direct push pairs")
    for (pusher, pushed), count in pushes.most_common(20):
        print(f"agent{pusher:02d}->agent{pushed:02d}: {count:3} times {push_times[(pusher, pushed)][:10]}")

    agent = args.agent
    print(f"\nAgent{agent:02d} event trace, goal={goals[agent]}")
    for t in args.events:
        if t <= 0 or t > makespan:
            continue
        prev = paths[agent][t - 1]
        cur = paths[agent][t]
        raw_candidates = neighbors(grid, prev) + [prev]
        prev_at = {paths[i][t - 1]: i for i in range(len(paths))}
        print(
            f"\nt={t} agent{agent:02d} {prev}->{cur} "
            f"dist={dists[agent][prev]}->{dists[agent][cur]} "
            f"prio_prev={priorities[t - 1][agent]:.2f}"
        )
        for idx, cand in enumerate(raw_candidates):
            owner = prev_at.get(cand)
            print(
                f"  cand#{idx} {cand} dist={dists[agent].get(cand, 999):2} "
                f"occ_prev={'agent' + str(owner) if owner is not None else '-':>7} "
                f"chosen={cand == cur}"
            )
        top = sorted(range(len(paths)), key=lambda i: priorities[t - 1][i], reverse=True)[:8]
        print("  top_priorities:", " ".join(f"a{i}:{priorities[t - 1][i]:.2f}/d{dists[i][paths[i][t - 1]]}" for i in top))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
