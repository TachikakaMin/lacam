#!/usr/bin/env python3
"""Validate ITA-CBS-style TAPF solutions.

Checks:
- every scheduled path starts at the input start
- every move is wait or 4-neighbor
- no vertex conflicts
- no edge-swap conflicts
- every agent ends at one of its own potentialGoals/goal locations
- final goal locations are unique by default
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

from tapf_schedule_io import load_yaml, load_schedule


Coord = Tuple[int, int]


def agent_index(name: str) -> int:
    if name.startswith("agent"):
        suffix = name[len("agent") :]
        if suffix.isdigit():
            return int(suffix)
    raise ValueError(f"cannot parse agent name: {name}")


def coord_from_list(value: Any) -> Coord:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"expected coordinate pair, got {value!r}")
    return int(value[0]), int(value[1])


def state_coord(state: Dict[str, Any]) -> Coord:
    return int(state["x"]), int(state["y"])


def state_at(path: List[Dict[str, Any]], t: int) -> Coord:
    current = path[0]
    for state in path:
        if int(state.get("t", 0)) > t:
            break
        current = state
    return state_coord(current)


def state_time(state: Dict[str, Any], fallback: int) -> int:
    return int(state.get("t", fallback))


def validate(input_yaml: Path, output_yaml: Path, require_unique_goals: bool) -> List[str]:
    instance = load_yaml(input_yaml)
    output = load_schedule(output_yaml)
    errors: List[str] = []

    input_agents = instance.get("agents") or []
    schedule = (output or {}).get("schedule") or {}

    if len(schedule) != len(input_agents):
        errors.append(
            f"agent count mismatch: input={len(input_agents)} schedule={len(schedule)}"
        )

    final_goals: Dict[Coord, str] = {}
    max_t = int((output.get("statistics") or {}).get("makespan", -1)) + 1
    scheduled_paths: Dict[str, List[Dict[str, Any]]] = {}

    for i, agent in enumerate(input_agents):
        name = agent.get("name", f"agent{i}")
        path = schedule.get(name)
        if path is None:
            errors.append(f"{name}: missing schedule")
            continue
        if not path:
            errors.append(f"{name}: empty schedule")
            continue
        scheduled_paths[name] = path
        path_end = max(state_time(state, idx) for idx, state in enumerate(path))
        max_t = max(max_t, path_end + 1)

        start = coord_from_list(agent["start"])
        if state_coord(path[0]) != start:
            errors.append(f"{name}: invalid start {state_coord(path[0])}, expected {start}")

        prev_t = -1
        for step_idx, state in enumerate(path):
            state_t = state_time(state, step_idx)
            if state_t <= prev_t:
                errors.append(f"{name}: non-increasing time at step {step_idx}")
                break
            prev_t = state_t

        for t in range(1, len(path)):
            prev = state_coord(path[t - 1])
            curr = state_coord(path[t])
            prev_time = state_time(path[t - 1], t - 1)
            curr_time = state_time(path[t], t)
            manhattan = abs(prev[0] - curr[0]) + abs(prev[1] - curr[1])
            if manhattan > 1:
                errors.append(
                    f"{name}: invalid move t={prev_time}->{curr_time}: {prev}->{curr}"
                )
                break

        raw_goals = agent.get("potentialGoals")
        if raw_goals is None:
            raw_goals = [agent["goal"]]
        allowed_goals = {coord_from_list(goal) for goal in raw_goals}
        final = state_coord(path[-1])
        if final not in allowed_goals:
            errors.append(
                f"{name}: final {final} is not in allowed goals {sorted(allowed_goals)}"
            )
        if require_unique_goals:
            other = final_goals.get(final)
            if other is not None:
                errors.append(f"{name}: final goal {final} duplicates {other}")
            final_goals[final] = name

    names = sorted(scheduled_paths.keys(), key=agent_index)
    for t in range(max_t):
        occupied: Dict[Coord, str] = {}
        for name in names:
            pos = state_at(scheduled_paths[name], t)
            other = occupied.get(pos)
            if other is not None:
                errors.append(f"vertex conflict t={t}: {other} and {name} at {pos}")
            occupied[pos] = name

        if t + 1 >= max_t:
            continue
        for idx, name_a in enumerate(names):
            a0 = state_at(scheduled_paths[name_a], t)
            a1 = state_at(scheduled_paths[name_a], t + 1)
            for name_b in names[idx + 1 :]:
                b0 = state_at(scheduled_paths[name_b], t)
                b1 = state_at(scheduled_paths[name_b], t + 1)
                if a0 == b1 and a1 == b0:
                    errors.append(
                        f"edge conflict t={t}: {name_a} {a0}->{a1}, "
                        f"{name_b} {b0}->{b1}"
                    )

    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_yaml", type=Path)
    parser.add_argument("output_yaml", type=Path)
    parser.add_argument(
        "--allow-duplicate-goals",
        action="store_true",
        help="Do not require final assigned goals to be unique.",
    )
    args = parser.parse_args()

    errors = validate(
        args.input_yaml,
        args.output_yaml,
        require_unique_goals=not args.allow_duplicate_goals,
    )
    if errors:
        print("INVALID")
        for error in errors:
            print(f"- {error}")
        return 1

    print("VALID")
    return 0


if __name__ == "__main__":
    sys.exit(main())
