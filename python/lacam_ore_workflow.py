#!/usr/bin/env python3
"""Event-driven ore/lifelong TAPF workflow for LaCAM-TAPF.

This mirrors the ITA-CBS ore workflow at a process level: each round converts the
current lifelong ore state into a one-shot TAPF instance, runs LaCAM-TAPF, then
executes the joint path until the first pickup/dropoff event.
"""

from __future__ import annotations

import argparse
import copy
import json
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml


Coord = Tuple[int, int]  # (row, col), same convention used by TAPF fixtures.
OCCUPIED_MAP_CHARS = {"@", "T", "O", "W"}


@dataclass
class AgentState:
    name: str
    pos: Coord
    is_droppingoff: bool
    past_path_cost: int
    current_hold_ore: int
    capacity: int
    potential_goal_ids: List[int]
    potential_dropoff_ids: List[int]
    current_target: Optional[Coord] = None
    current_service_ore: Optional[Coord] = None


@dataclass
class ScenarioState:
    scenario_path: Optional[Path]
    base_dir: Path
    map_spec: Any
    grid: List[List[int]]
    ore_points: List[Coord]
    ore_amounts: List[int]
    dropoff_points: List[Coord]
    agents: List[AgentState]


def _normalize_coord(node: Any) -> Coord:
    if not isinstance(node, Sequence) or isinstance(node, (str, bytes)) or len(node) != 2:
        raise ValueError(f"Invalid coordinate node: {node}")
    return int(node[0]), int(node[1])


def _load_map_from_file(map_path: Path) -> List[List[int]]:
    if not map_path.exists():
        raise FileNotFoundError(f"Map file not found: {map_path}")
    lines = [line.rstrip("\n") for line in map_path.read_text(encoding="utf-8").splitlines()]
    if len(lines) < 5:
        raise ValueError(f"Invalid .map format: {map_path}")
    height = int(lines[1].split()[1])
    width = int(lines[2].split()[1])
    raw = lines[4:]
    if len(raw) != height:
        raise ValueError(f"Map height mismatch in {map_path}: header={height}, rows={len(raw)}")
    grid = [[0 for _ in range(width)] for _ in range(height)]
    for r, line in enumerate(raw):
        if len(line) != width:
            raise ValueError(f"Map width mismatch in {map_path} at row {r}")
        for c, ch in enumerate(line):
            grid[r][c] = 1 if ch in OCCUPIED_MAP_CHARS else 0
    return grid


def _load_map_from_spec(map_spec: Any, base_dir: Path) -> List[List[int]]:
    if isinstance(map_spec, dict):
        dims = map_spec.get("dimensions")
        if not isinstance(dims, Sequence) or len(dims) != 2:
            raise ValueError("Inline map requires dimensions: [rows, cols]")
        rows, cols = int(dims[0]), int(dims[1])
        grid = [[0 for _ in range(cols)] for _ in range(rows)]
        for ob in map_spec.get("obstacles", []):
            r, c = _normalize_coord(ob)
            if 0 <= r < rows and 0 <= c < cols:
                grid[r][c] = 1
        return grid
    if isinstance(map_spec, str):
        map_path = Path(map_spec)
        if not map_path.is_absolute():
            map_path = base_dir / map_path
        return _load_map_from_file(map_path)
    raise ValueError("mapinfo.map must be a string or inline map object")


def load_scenario(path: Path) -> ScenarioState:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Scenario YAML must be a map/object")
    mapinfo = data.get("mapinfo", data)
    if "map" not in mapinfo:
        raise ValueError("Scenario missing mapinfo.map")

    ore_points = [_normalize_coord(p) for p in mapinfo.get("potentialGoals", [])]
    ore_amounts = [max(0, int(v)) for v in mapinfo.get("potentialGoalsOre", [])]
    if len(ore_amounts) < len(ore_points):
        ore_amounts.extend([0] * (len(ore_points) - len(ore_amounts)))
    elif len(ore_amounts) > len(ore_points):
        ore_amounts = ore_amounts[: len(ore_points)]
    dropoff_points = [_normalize_coord(p) for p in mapinfo.get("potentialDropoffGoals", [])]

    agents = []
    for i, node in enumerate(data.get("agents", [])):
        potential_goal_ids = [int(x) for x in node.get("potentialGoals", list(range(len(ore_points))))]
        potential_dropoff_ids = [int(x) for x in node.get("potentialDropoffGoals", list(range(len(dropoff_points))))]
        target = None
        if node.get("currentTarget") is not None:
            target = _normalize_coord(node["currentTarget"])
        service_ore = None
        if node.get("currentServiceOre") is not None:
            service_ore = _normalize_coord(node["currentServiceOre"])
        agents.append(
            AgentState(
                name=str(node.get("name", f"agent{i}")),
                pos=_normalize_coord(node.get("start", [0, 0])),
                is_droppingoff=bool(node.get("isDroppingoff", False)),
                past_path_cost=int(node.get("pastPathCost", 0)),
                current_hold_ore=max(0, int(node.get("currentHoldOre", 0))),
                capacity=max(0, int(node.get("capacity", 0))),
                potential_goal_ids=potential_goal_ids,
                potential_dropoff_ids=potential_dropoff_ids,
                current_target=target,
                current_service_ore=service_ore,
            )
        )

    base_dir = path.parent.resolve()
    grid = _load_map_from_spec(mapinfo["map"], base_dir)
    return ScenarioState(
        scenario_path=path.resolve(),
        base_dir=base_dir,
        map_spec=copy.deepcopy(mapinfo["map"]),
        grid=grid,
        ore_points=ore_points,
        ore_amounts=ore_amounts,
        dropoff_points=dropoff_points,
        agents=agents,
    )


def _scenario_to_yaml_dict(state: ScenarioState) -> Dict[str, Any]:
    return {
        "mapinfo": {
            "map": copy.deepcopy(state.map_spec),
            "potentialGoals": [[r, c] for r, c in state.ore_points],
            "potentialGoalsOre": [int(v) for v in state.ore_amounts],
            "potentialDropoffGoals": [[r, c] for r, c in state.dropoff_points],
        },
        "agents": [
            {
                "name": a.name,
                "potentialGoals": [int(x) for x in a.potential_goal_ids],
                "start": [a.pos[0], a.pos[1]],
                "isDroppingoff": bool(a.is_droppingoff),
                "pastPathCost": int(a.past_path_cost),
                "currentHoldOre": int(a.current_hold_ore),
                "capacity": int(a.capacity),
                "potentialDropoffGoals": [int(x) for x in a.potential_dropoff_ids],
                **({"currentTarget": [a.current_target[0], a.current_target[1]]} if a.current_target else {}),
                **({"currentServiceOre": [a.current_service_ore[0], a.current_service_ore[1]]} if a.current_service_ore else {}),
            }
            for a in state.agents
        ],
    }


def save_scenario(path: Path, state: ScenarioState) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(_scenario_to_yaml_dict(state), sort_keys=False), encoding="utf-8")


def _write_inline_map_as_movingai(path: Path, grid: List[List[int]]) -> None:
    rows = len(grid)
    cols = len(grid[0]) if rows else 0
    lines = ["type octile", f"height {rows}", f"width {cols}", "map"]
    for r in range(rows):
        lines.append("".join("@" if grid[r][c] else "." for c in range(cols)))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _round_map_name(state: ScenarioState, work_dir: Path) -> str:
    if isinstance(state.map_spec, str):
        map_path = Path(state.map_spec)
        if not map_path.is_absolute():
            map_path = state.base_dir / map_path
        return str(map_path.resolve())
    map_path = work_dir / "inline_map.map"
    _write_inline_map_as_movingai(map_path, state.grid)
    return str(map_path.resolve())


def _grid_distance(state: ScenarioState, start: Coord, goals: Sequence[Coord]) -> int:
    goal_set = set(goals)
    if not goal_set:
        return 10**9
    if start in goal_set:
        return 0
    rows = len(state.grid)
    cols = len(state.grid[0]) if rows else 0
    queue = [(start, 0)]
    seen = {start}
    head = 0
    while head < len(queue):
        (r, c), d = queue[head]
        head += 1
        for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if state.grid[nr][nc] or (nr, nc) in seen:
                continue
            if (nr, nc) in goal_set:
                return d + 1
            seen.add((nr, nc))
            queue.append(((nr, nc), d + 1))
    return 10**9


def _passable(state: ScenarioState, pos: Coord) -> bool:
    r, c = pos
    rows = len(state.grid)
    cols = len(state.grid[0]) if rows else 0
    return 0 <= r < rows and 0 <= c < cols and not state.grid[r][c]


def _dropoff_targets(agent: AgentState, state: ScenarioState) -> List[Coord]:
    return [
        state.dropoff_points[i]
        for i in agent.potential_dropoff_ids
        if 0 <= i < len(state.dropoff_points)
    ]


def _pickup_targets(agent: AgentState, state: ScenarioState) -> List[Coord]:
    return [
        state.ore_points[i]
        for i in agent.potential_goal_ids
        if 0 <= i < len(state.ore_points) and state.ore_amounts[i] > 0
    ]


def _staging_cells_for_ore(state: ScenarioState, ore: Coord, limit: int) -> List[Coord]:
    if limit <= 0:
        return []
    rows = len(state.grid)
    cols = len(state.grid[0]) if rows else 0
    queue = [(ore, 0)]
    seen = {ore}
    cells: List[Coord] = []
    head = 0
    while head < len(queue) and len(cells) < limit:
        (r, c), _d = queue[head]
        head += 1
        if _passable(state, (r, c)):
            cells.append((r, c))
        for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
            if not (0 <= nr < rows and 0 <= nc < cols) or (nr, nc) in seen:
                continue
            if state.grid[nr][nc]:
                continue
            seen.add((nr, nc))
            queue.append(((nr, nc), _d + 1))
    return cells


def _nearest_free_cell(state: ScenarioState, start: Coord, reserved: set[Coord]) -> Coord:
    rows = len(state.grid)
    cols = len(state.grid[0]) if rows else 0
    queue = [start]
    seen = {start}
    head = 0
    while head < len(queue):
        r, c = queue[head]
        head += 1
        if _passable(state, (r, c)) and (r, c) not in reserved:
            return (r, c)
        for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
            if not (0 <= nr < rows and 0 <= nc < cols) or (nr, nc) in seen:
                continue
            if state.grid[nr][nc]:
                continue
            seen.add((nr, nc))
            queue.append((nr, nc))
    return start


def _best_dropoff_distance_from_ore(agent: AgentState, state: ScenarioState, ore: Coord) -> int:
    return _grid_distance(state, ore, _dropoff_targets(agent, state))


def _pickup_score(agent: AgentState, state: ScenarioState, ore: Coord, goal: Coord) -> int:
    ore_idx = state.ore_points.index(ore)
    service_amount = min(max(1, agent.capacity), max(0, state.ore_amounts[ore_idx]))
    to_goal = _grid_distance(state, agent.pos, [goal])
    goal_to_ore = _grid_distance(state, goal, [ore])
    ore_to_dropoff = _best_dropoff_distance_from_ore(agent, state, ore)
    if to_goal >= 10**9 or goal_to_ore >= 10**9 or ore_to_dropoff >= 10**9:
        return 10**9
    commitment_bonus = 0
    if agent.current_service_ore == ore:
        commitment_bonus += 8
    if agent.current_target == goal:
        commitment_bonus += 4
    return to_goal + goal_to_ore + ore_to_dropoff - service_amount - commitment_bonus


def _empty_pickup_agents(state: ScenarioState) -> List[int]:
    return [
        i
        for i, agent in enumerate(state.agents)
        if not agent.is_droppingoff
        and agent.current_hold_ore <= 0
        and agent.capacity > 0
        and _pickup_targets(agent, state)
    ]


def _primary_pickup_agents(state: ScenarioState, empty_agents: Sequence[int]) -> set[int]:
    active_ore = {
        p
        for i, p in enumerate(state.ore_points)
        if i < len(state.ore_amounts) and state.ore_amounts[i] > 0
    }
    candidates = []
    for i in empty_agents:
        agent = state.agents[i]
        targets = _pickup_targets(agent, state)
        candidates.append((_grid_distance(state, agent.pos, targets), i))
    candidates.sort()
    return {i for _, i in candidates[: min(len(candidates), len(active_ore))]}


def _staging_pickup_plan(state: ScenarioState, primary_agents: set[int], empty_agents: Sequence[int]) -> Dict[int, Dict[str, Any]]:
    loaded_count = sum(1 for agent in state.agents if agent.is_droppingoff or agent.current_hold_ore > 0)
    if loaded_count >= 3:
        return {}
    extra_agents = [i for i in empty_agents if i not in primary_agents]
    if not extra_agents:
        return {}

    blocked_goals = {state.agents[i].pos for i in range(len(state.agents)) if i not in extra_agents}
    slots: List[Dict[str, Any]] = []
    for ore_idx, ore in enumerate(state.ore_points):
        ore_left = state.ore_amounts[ore_idx] if ore_idx < len(state.ore_amounts) else 0
        if ore_left <= 0:
            continue
        interested = [i for i in extra_agents if ore_idx in state.agents[i].potential_goal_ids]
        if not interested:
            continue
        max_capacity = max((state.agents[i].capacity for i in interested), default=1)
        if ore_left < 4 * max_capacity:
            continue
        for rank, goal in enumerate(_staging_cells_for_ore(state, ore, 6)):
            if goal == ore or goal in blocked_goals:
                continue
            slots.append({"ore": ore, "goal": goal, "rank": rank})
            break

    candidates = []
    for i in extra_agents:
        agent = state.agents[i]
        for slot_idx, slot in enumerate(slots):
            ore = slot["ore"]
            ore_idx = state.ore_points.index(ore)
            if ore_idx not in agent.potential_goal_ids:
                continue
            score = _pickup_score(agent, state, ore, slot["goal"]) + int(slot["rank"])
            if score < 10**9:
                candidates.append((score, 0 if agent.current_service_ore == ore else 1, i, slot_idx))
    candidates.sort()

    max_extra = max(0, min(2, 4 - loaded_count))
    used_agents = set()
    used_slots = set()
    plan: Dict[int, Dict[str, Any]] = {}
    for _score, _switch, agent_idx, slot_idx in candidates:
        if len(plan) >= max_extra:
            break
        if agent_idx in used_agents or slot_idx in used_slots:
            continue
        used_agents.add(agent_idx)
        used_slots.add(slot_idx)
        plan[agent_idx] = slots[slot_idx]
    return plan


def _agent_targets(
    agent_index: int,
    agent: AgentState,
    state: ScenarioState,
    primary_pickup_agents: set[int],
    staging_plan: Dict[int, Dict[str, Any]],
    reserved_targets: set[Coord],
) -> Tuple[List[Coord], str, Optional[Coord]]:
    if agent.is_droppingoff or agent.current_hold_ore > 0 or agent.current_hold_ore >= agent.capacity > 0:
        targets = _dropoff_targets(agent, state)
        return targets or [_nearest_free_cell(state, agent.pos, reserved_targets)], "dropoff", None
    slot = staging_plan.get(agent_index)
    if slot is not None:
        return [slot["goal"]], "pickup", slot["ore"]
    if agent_index in primary_pickup_agents:
        targets = _pickup_targets(agent, state)
        return targets or [_nearest_free_cell(state, agent.pos, reserved_targets)], "pickup", None
    return [_nearest_free_cell(state, agent.pos, reserved_targets)], "idle", None


def write_round_tapf_input(path: Path, state: ScenarioState, work_dir: Path) -> List[List[Coord]]:
    map_name = _round_map_name(state, work_dir)
    planned_targets = []
    agents_yaml = []
    empty_agents = _empty_pickup_agents(state)
    primary_pickup_agents = _primary_pickup_agents(state, empty_agents)
    staging_plan = _staging_pickup_plan(state, primary_pickup_agents, empty_agents)
    reserved_targets = {slot["goal"] for slot in staging_plan.values()}
    for i, agent in enumerate(state.agents):
        targets, mode, service_ore = _agent_targets(
            i, agent, state, primary_pickup_agents, staging_plan, reserved_targets
        )
        if mode != "pickup" or service_ore is None:
            reserved_targets.update(targets)
        planned_targets.append(targets)
        if service_ore is not None:
            agent.current_service_ore = service_ore
        elif mode != "pickup":
            agent.current_service_ore = None
        agents_yaml.append(
            {
                "name": agent.name or f"agent{i}",
                "start": [agent.pos[0], agent.pos[1]],
                "potentialGoals": [[r, c] for r, c in targets],
                "lifelongMode": mode,
                "currentHoldOre": int(agent.current_hold_ore),
                "capacity": int(agent.capacity),
                **({"serviceOre": [service_ore[0], service_ore[1]]} if service_ore else {}),
            }
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump({"map": map_name, "agents": agents_yaml}, sort_keys=False), encoding="utf-8")
    return planned_targets

def _expand_schedule(seq: Sequence[Dict[str, Any]], fallback: Coord) -> List[Coord]:
    if not seq:
        return [fallback]
    entries = []
    for idx, node in enumerate(seq):
        t = int(node.get("t", idx))
        entries.append((t, (int(node["x"]), int(node["y"]))))
    entries.sort(key=lambda x: x[0])
    path: List[Coord] = []
    current = fallback
    next_expected = 0
    for t, pos in entries:
        while next_expected < t:
            path.append(current)
            next_expected += 1
        path.append(pos)
        current = pos
        next_expected = t + 1
    return path or [fallback]


def _parse_solver_output(out_path: Path, fallback_positions: List[Coord]) -> Tuple[List[List[Coord]], List[Optional[Coord]], Any, int, bool]:
    data = yaml.safe_load(out_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Invalid solver output YAML: {out_path}")
    schedule = data.get("schedule") or {}
    assignment_nodes = data.get("assignments") or {}
    paths = []
    assigned_goals: List[Optional[Coord]] = []
    for i, fallback in enumerate(fallback_positions):
        paths.append(_expand_schedule(schedule.get(f"agent{i}", []), fallback))
        node = assignment_nodes.get(f"agent{i}")
        assigned_goals.append((int(node["x"]), int(node["y"])) if isinstance(node, dict) and "x" in node and "y" in node else None)
    stats = data.get("statistics") or {}
    team_size = int(stats.get("teamSize", len(schedule) if schedule else 0))
    return paths, assigned_goals, stats.get("cost"), team_size, bool(schedule)


def _coord_to_list(pos: Optional[Coord]) -> Optional[List[int]]:
    return [int(pos[0]), int(pos[1])] if pos is not None else None


def _first_event_time_for_agent(
    agent: AgentState,
    path: List[Coord],
    assigned_goal: Optional[Coord],
    ore_index: Dict[Coord, int],
    ore_amounts: Sequence[int],
    dropoff_set: set[Coord],
) -> Tuple[Optional[int], Optional[str], Optional[Coord]]:
    if not path or assigned_goal is None:
        return None, None, None
    if agent.is_droppingoff or agent.current_hold_ore > 0:
        kind = "dropoff" if assigned_goal in dropoff_set else "dropoff_arrival"
    else:
        ore_idx = ore_index.get(assigned_goal)
        if ore_idx is not None and ore_amounts[ore_idx] > 0:
            kind = "pickup"
        elif agent.current_service_ore is not None:
            kind = "pickup_arrival"
        else:
            return None, None, None
    for t, pos in enumerate(path):
        if pos == assigned_goal:
            if t == 0 and kind in {"pickup_arrival", "dropoff_arrival"}:
                return None, None, None
            return t, kind, pos
    return None, None, None


def _first_collision_time(paths: List[List[Coord]]) -> Optional[int]:
    if not paths:
        return None
    max_t = max(len(p) for p in paths)

    def at(path: List[Coord], t: int) -> Coord:
        return path[min(t, len(path) - 1)]

    for t in range(max_t):
        occupied: Dict[Coord, int] = {}
        for i, path in enumerate(paths):
            pos = at(path, t)
            if pos in occupied:
                return t
            occupied[pos] = i
        for i in range(len(paths)):
            for j in range(i + 1, len(paths)):
                if at(paths[i], t) == at(paths[j], t + 1) and at(paths[i], t + 1) == at(paths[j], t):
                    return t
    return None


def _run_lacam_solver(
    binary: Path,
    input_yaml: Path,
    output_yaml: Path,
    map_dir: Path,
    time_limit: float,
    timeout_s: float,
    anytime: bool,
) -> str:
    cmd = [str(binary), str(input_yaml), str(map_dir), str(time_limit), str(output_yaml), "1" if anytime else "0"]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=max(timeout_s, time_limit + 2.0))
    output = (proc.stdout or "") + (proc.stderr or "")
    if proc.returncode != 0:
        raise RuntimeError(f"LaCAM-TAPF failed (code {proc.returncode}).\nCommand: {' '.join(cmd)}\n{output}")
    if not output_yaml.exists():
        raise RuntimeError(f"LaCAM-TAPF finished without writing output: {output_yaml}\n{output}")
    return output

def _write_round_assignment_file(
    path: Path,
    rid: int,
    sim_time: int,
    solver_cost: Any,
    team_size: int,
    has_schedule: bool,
    agents: Sequence[AgentState],
    paths: Sequence[List[Coord]],
    planned_targets: Sequence[Sequence[Coord]],
    candidate_events: Sequence[Dict[str, Any]],
) -> None:
    event_by_agent = {int(e["agent"]): e for e in candidate_events}
    assignments = []
    for i, agent in enumerate(agents):
        evt = event_by_agent.get(i)
        path_i = paths[i] if i < len(paths) else []
        assignments.append(
            {
                "agent": i,
                "name": agent.name,
                "mode": "dropoff" if agent.is_droppingoff or agent.current_hold_ore > 0 else "pickup",
                "start": [agent.pos[0], agent.pos[1]],
                "candidate_goals": [[r, c] for r, c in planned_targets[i]] if i < len(planned_targets) else [],
                "event_type": evt.get("kind") if evt else None,
                "event_time": int(evt["time"]) if evt else None,
                "event_pos": _coord_to_list(evt.get("pos")) if evt else None,
                "assigned_goal": _coord_to_list(evt.get("assigned_goal")) if evt else None,
                "service_ore": _coord_to_list(agent.current_service_ore),
                "path_nodes": len(path_i),
                "path_end": _coord_to_list(path_i[-1]) if path_i else None,
            }
        )
    payload = {
        "round": rid,
        "time_start": sim_time,
        "has_schedule": has_schedule,
        "team_size": team_size,
        "solver_cost": solver_cost,
        "assignments": assignments,
    }
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def run_event_simulation(
    scenario: ScenarioState,
    lacam_binary: Path,
    map_dir: Path,
    max_rounds: int,
    max_steps: int,
    work_dir: Path,
    keep_round_files: bool,
    timeout_s: float,
    time_limit: float,
    anytime: bool,
    verbose: bool,
) -> Dict[str, Any]:
    if not lacam_binary.exists():
        raise FileNotFoundError(f"LaCAM binary not found: {lacam_binary}")
    state = copy.deepcopy(scenario)
    if isinstance(state.map_spec, str):
        map_path = Path(state.map_spec)
        if not map_path.is_absolute():
            state.map_spec = str((state.base_dir / map_path).resolve())
    work_dir.mkdir(parents=True, exist_ok=True)

    trajectories = {str(i): [[a.pos[0], a.pos[1]]] for i, a in enumerate(state.agents)}
    rounds: List[Dict[str, Any]] = []
    ore_checkpoints = [{"time": 0, "ore": [int(v) for v in state.ore_amounts]}]
    sim_time = 0
    total_delivered_ore = 0
    stop_reason = "max_rounds"

    for rid in range(max_rounds):
        if sum(state.ore_amounts) <= 0 and all(a.current_hold_ore <= 0 for a in state.agents):
            stop_reason = "all_ore_finished"
            break
        if sim_time >= max_steps:
            stop_reason = "max_steps"
            break

        scenario_file = work_dir / f"round_{rid:04d}_state.yaml"
        in_file = work_dir / f"round_{rid:04d}_tapf.yaml"
        out_file = work_dir / f"round_{rid:04d}_output.yaml"
        assign_file = work_dir / f"round_{rid:04d}_assignment.yaml"
        save_scenario(scenario_file, state)
        planned_targets = write_round_tapf_input(in_file, state, work_dir)

        if verbose:
            print(f"[round {rid}] t={sim_time} ore_left={sum(state.ore_amounts)} loaded={sum(1 for a in state.agents if a.current_hold_ore > 0 or a.is_droppingoff)}")
        solver_log = _run_lacam_solver(lacam_binary, in_file, out_file, map_dir, time_limit, timeout_s, anytime)
        paths, assigned_goals, solver_cost, team_size, has_schedule = _parse_solver_output(out_file, [a.pos for a in state.agents])

        if not has_schedule:
            stop_reason = "solver_no_schedule"
            rounds.append({"round": rid, "time_start": sim_time, "time_end": sim_time, "delta": 0, "events": [], "ore_after": list(state.ore_amounts), "solver_log": solver_log[-2000:]})
            break

        ore_index = {p: i for i, p in enumerate(state.ore_points)}
        dropoff_set = set(state.dropoff_points)
        candidate_events = []
        for i, (agent, path) in enumerate(zip(state.agents, paths)):
            assigned_goal = assigned_goals[i] if i < len(assigned_goals) else None
            t, kind, pos = _first_event_time_for_agent(agent, path, assigned_goal, ore_index, state.ore_amounts, dropoff_set)
            agent.current_target = assigned_goal
            if t is not None:
                candidate_events.append({"agent": i, "time": t, "kind": kind, "pos": pos, "assigned_goal": assigned_goal})
        _write_round_assignment_file(assign_file, rid, sim_time, solver_cost, team_size, has_schedule, state.agents, paths, planned_targets, candidate_events)

        if not candidate_events:
            stop_reason = "no_future_event"
            rounds.append({"round": rid, "time_start": sim_time, "time_end": sim_time, "delta": 0, "solver_cost": solver_cost, "events": [], "ore_after": list(state.ore_amounts), "solver_log": solver_log[-2000:]})
            break

        delta = min(int(e["time"]) for e in candidate_events)
        if sim_time + delta > max_steps:
            delta = max_steps - sim_time
        if delta < 0:
            stop_reason = "zero_delta"
            break

        for i, (agent, path) in enumerate(zip(state.agents, paths)):
            for step in range(1, delta + 1):
                p = path[min(step, len(path) - 1)]
                trajectories[str(i)].append([p[0], p[1]])
            end_pos = path[min(delta, len(path) - 1)]
            agent.pos = end_pos
            agent.past_path_cost += delta

        round_events = []
        executed_events = sorted(
            (e for e in candidate_events if int(e["time"]) == delta),
            key=lambda e: (int(e["time"]), 0 if e.get("kind") == "dropoff" else 1, int(e["agent"])),
        )
        for e in executed_events:
            i = int(e["agent"])
            agent = state.agents[i]
            pos = e["pos"]
            event_time = int(e["time"])
            if e["kind"] == "pickup":
                ore_idx = ore_index[pos]
                available = max(0, state.ore_amounts[ore_idx])
                free_cap = max(0, agent.capacity - agent.current_hold_ore)
                take = min(available, free_cap)
                if take > 0:
                    state.ore_amounts[ore_idx] -= take
                    agent.current_hold_ore += take
                    agent.is_droppingoff = True
                agent.current_target = None
                agent.current_service_ore = None
                round_events.append({"agent": i, "time": event_time, "type": "pickup", "pos": [pos[0], pos[1]], "amount": int(take), "ore_remaining_at_point": int(state.ore_amounts[ore_idx])})
            elif e["kind"] == "dropoff":
                delivered = max(0, agent.current_hold_ore)
                total_delivered_ore += delivered
                agent.current_hold_ore = 0
                agent.is_droppingoff = False
                agent.past_path_cost = 0
                agent.current_target = None
                agent.current_service_ore = None
                round_events.append({"agent": i, "time": event_time, "type": "dropoff", "pos": [pos[0], pos[1]], "amount": int(delivered)})
            elif e["kind"] == "pickup_arrival":
                agent.current_target = None
                round_events.append({"agent": i, "time": event_time, "type": "pickup_arrival", "pos": [pos[0], pos[1]], "service_ore": [agent.current_service_ore[0], agent.current_service_ore[1]] if agent.current_service_ore else None})
            elif e["kind"] == "dropoff_arrival":
                agent.current_target = None
                round_events.append({"agent": i, "time": event_time, "type": "dropoff_arrival", "pos": [pos[0], pos[1]]})

        sim_time += delta
        ore_checkpoints.append({"time": sim_time, "ore": [int(v) for v in state.ore_amounts]})
        rounds.append({"round": rid, "time_start": sim_time - delta, "time_end": sim_time, "delta": int(delta), "solver_cost": solver_cost, "events": round_events, "ore_after": [int(v) for v in state.ore_amounts], "solver_log": solver_log[-2000:]})

        if not keep_round_files:
            for p in (scenario_file, in_file, out_file):
                try:
                    p.unlink()
                except FileNotFoundError:
                    pass

    return {
        "source_scenario": str(scenario.scenario_path) if scenario.scenario_path else None,
        "lacam_binary": str(lacam_binary),
        "final_time": int(sim_time),
        "stop_reason": stop_reason,
        "total_delivered_ore": int(total_delivered_ore),
        "ore_points": [[r, c] for r, c in state.ore_points],
        "dropoff_points": [[r, c] for r, c in state.dropoff_points],
        "ore_checkpoints": ore_checkpoints,
        "map_spec": copy.deepcopy(state.map_spec),
        "rounds": rounds,
        "agents": {
            str(i): {
                "name": a.name,
                "trajectory": trajectories[str(i)],
                "final": {
                    "pos": [a.pos[0], a.pos[1]],
                    "isDroppingoff": bool(a.is_droppingoff),
                    "pastPathCost": int(a.past_path_cost),
                    "currentHoldOre": int(a.current_hold_ore),
                    "capacity": int(a.capacity),
                    "currentTarget": [a.current_target[0], a.current_target[1]] if a.current_target else None,
                    "currentServiceOre": [a.current_service_ore[0], a.current_service_ore[1]] if a.current_service_ore else None,
                },
            }
            for i, a in enumerate(state.agents)
        },
    }


def _ore_at_time(checkpoints: List[Dict[str, Any]], t: int) -> List[int]:
    if not checkpoints:
        return []
    best = checkpoints[0].get("ore", [])
    for checkpoint in checkpoints:
        if int(checkpoint.get("time", 0)) <= t:
            best = checkpoint.get("ore", [])
        else:
            break
    return [int(x) for x in best]


def _load_grid_for_replay(data: Dict[str, Any], sim_output_path: Path) -> List[List[int]]:
    map_spec = data.get("map_spec")
    if map_spec is None:
        raise ValueError("Simulation output missing map_spec")
    base_dir = sim_output_path.parent
    if isinstance(map_spec, str):
        map_path = Path(map_spec)
        if not map_path.is_absolute() and data.get("source_scenario"):
            scenario_parent = Path(str(data["source_scenario"])).parent
            candidate = scenario_parent / map_path
            if candidate.exists():
                map_spec = str(candidate)
    return _load_map_from_spec(map_spec, base_dir)


def _replay_data(sim_output_path: Path) -> Tuple[Dict[str, Any], List[List[int]], Dict[int, List[Coord]]]:
    data = json.loads(sim_output_path.read_text(encoding="utf-8"))
    grid = _load_grid_for_replay(data, sim_output_path)
    trajectories = {
        int(agent_id): [tuple(_normalize_coord(pos)) for pos in payload.get("trajectory", [])]
        for agent_id, payload in data.get("agents", {}).items()
    }
    return data, grid, trajectories


def _draw_replay_canvas(canvas: Any, data: Dict[str, Any], grid: List[List[int]], trajectories: Dict[int, List[Coord]], t: int, cell: int, margin: int) -> None:
    canvas.delete("all")
    rows = len(grid)
    cols = len(grid[0]) if rows else 0
    ore_points = [tuple(_normalize_coord(p)) for p in data.get("ore_points", [])]
    dropoff_points = {tuple(_normalize_coord(p)) for p in data.get("dropoff_points", [])}
    ore_now = _ore_at_time(data.get("ore_checkpoints", []), t)
    ore_by_pos = {ore_points[i]: ore_now[i] if i < len(ore_now) else 0 for i in range(len(ore_points))}

    def cell_box(r: int, c: int) -> Tuple[int, int, int, int]:
        x0 = margin + c * cell
        y0 = margin + r * cell
        return x0, y0, x0 + cell, y0 + cell

    for r in range(rows):
        for c in range(cols):
            x0, y0, x1, y1 = cell_box(r, c)
            fill = "#303030" if grid[r][c] else "#ffffff"
            canvas.create_rectangle(x0, y0, x1, y1, fill=fill, outline="#c8c8c8")

    for r, c in dropoff_points:
        x0, y0, x1, y1 = cell_box(r, c)
        canvas.create_rectangle(x0 + 4, y0 + 4, x1 - 4, y1 - 4, fill="#40b37f", outline="")

    for (r, c), amount in ore_by_pos.items():
        x0, y0, x1, y1 = cell_box(r, c)
        fill = "#f39c12" if amount > 0 else "#f8d9a0"
        canvas.create_oval(x0 + 5, y0 + 5, x1 - 5, y1 - 5, fill=fill, outline="#8a5a00")
        canvas.create_text((x0 + x1) // 2, (y0 + y1) // 2, text=str(amount), fill="black")

    palette = ["#3498db", "#e74c3c", "#9b59b6", "#16a085", "#d35400", "#2c3e50", "#27ae60", "#c0392b"]
    for aid in sorted(trajectories):
        traj = trajectories[aid]
        if not traj:
            continue
        r, c = traj[min(t, len(traj) - 1)]
        x0, y0, x1, y1 = cell_box(r, c)
        color = palette[aid % len(palette)]
        canvas.create_oval(x0 + 6, y0 + 6, x1 - 6, y1 - 6, fill=color, outline="")
        canvas.create_text((x0 + x1) // 2, (y0 + y1) // 2, text=str(aid), fill="white")

    canvas.create_text(
        margin,
        max(10, margin // 2),
        anchor="w",
        text=f"t={t}  delivered={data.get('total_delivered_ore', 0)}  stop={data.get('stop_reason', '')}",
        fill="#222222",
    )


def replay_with_tk(sim_output_path: Path, cell: int = 26, speed_ms: int = 150) -> None:
    import tkinter as tk

    data, grid, trajectories = _replay_data(sim_output_path)
    rows = len(grid)
    cols = len(grid[0]) if rows else 0
    margin = 20
    max_t = max((len(traj) for traj in trajectories.values()), default=1) - 1

    root = tk.Tk()
    root.title("LaCAM Lifelong Ore Replay")
    canvas = tk.Canvas(root, width=margin * 2 + cols * cell, height=margin * 2 + rows * cell + 8, bg="#f8f8f8")
    canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    controls = tk.Frame(root)
    controls.pack(side=tk.BOTTOM, fill=tk.X)
    t_var = tk.IntVar(value=0)
    playing = {"value": False}
    after_id = {"value": None}

    def draw() -> None:
        _draw_replay_canvas(canvas, data, grid, trajectories, t_var.get(), cell, margin)

    def tick() -> None:
        if not playing["value"]:
            return
        if t_var.get() >= max_t:
            playing["value"] = False
            return
        t_var.set(t_var.get() + 1)
        draw()
        after_id["value"] = root.after(speed_ms, tick)

    def pause() -> None:
        playing["value"] = False
        if after_id["value"] is not None:
            root.after_cancel(after_id["value"])
            after_id["value"] = None

    def play() -> None:
        if playing["value"]:
            return
        playing["value"] = True
        tick()

    def step(delta: int) -> None:
        pause()
        t_var.set(max(0, min(max_t, t_var.get() + delta)))
        draw()

    tk.Button(controls, text="Play", command=play).pack(side=tk.LEFT)
    tk.Button(controls, text="Pause", command=pause).pack(side=tk.LEFT)
    tk.Button(controls, text="<", command=lambda: step(-1)).pack(side=tk.LEFT)
    tk.Button(controls, text=">", command=lambda: step(1)).pack(side=tk.LEFT)
    tk.Scale(controls, from_=0, to=max_t, orient=tk.HORIZONTAL, variable=t_var, command=lambda _v: draw()).pack(side=tk.LEFT, fill=tk.X, expand=True)

    draw()
    root.mainloop()


def write_snapshot(sim_output_path: Path, output_path: Path, t: int, cell: int = 28) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    data, grid, trajectories = _replay_data(sim_output_path)
    rows = len(grid)
    cols = len(grid[0]) if rows else 0
    ore_points = [tuple(_normalize_coord(p)) for p in data.get("ore_points", [])]
    dropoff_points = {tuple(_normalize_coord(p)) for p in data.get("dropoff_points", [])}
    ore_now = _ore_at_time(data.get("ore_checkpoints", []), t)
    ore_by_pos = {ore_points[i]: ore_now[i] if i < len(ore_now) else 0 for i in range(len(ore_points))}

    fig_w = max(4, cols * cell / 110)
    fig_h = max(4, rows * cell / 110)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    ax.set_xlim(0, cols)
    ax.set_ylim(rows, 0)
    ax.set_aspect("equal")
    ax.set_xticks(range(cols + 1))
    ax.set_yticks(range(rows + 1))
    ax.grid(color="#cccccc", linewidth=0.4)
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

    for r in range(rows):
        for c in range(cols):
            if grid[r][c]:
                ax.add_patch(Rectangle((c, r), 1, 1, color="#303030"))
    for r, c in dropoff_points:
        ax.add_patch(Rectangle((c + 0.12, r + 0.12), 0.76, 0.76, color="#40b37f"))
    for (r, c), amount in ore_by_pos.items():
        ax.add_patch(Circle((c + 0.5, r + 0.5), 0.34, color="#f39c12" if amount > 0 else "#f8d9a0"))
        ax.text(c + 0.5, r + 0.5, str(amount), ha="center", va="center", fontsize=8, color="black")

    palette = ["#3498db", "#e74c3c", "#9b59b6", "#16a085", "#d35400", "#2c3e50", "#27ae60", "#c0392b"]
    for aid in sorted(trajectories):
        traj = trajectories[aid]
        if not traj:
            continue
        r, c = traj[min(t, len(traj) - 1)]
        ax.add_patch(Circle((c + 0.5, r + 0.5), 0.28, color=palette[aid % len(palette)]))
        ax.text(c + 0.5, r + 0.5, str(aid), ha="center", va="center", fontsize=7, color="white")

    ax.set_title(f"LaCAM lifelong replay t={t}, delivered={data.get('total_delivered_ore', 0)}, stop={data.get('stop_reason', '')}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, bbox_inches="tight", dpi=160)
    plt.close(fig)


def replay(args: argparse.Namespace) -> int:
    replay_with_tk(args.input, cell=args.cell, speed_ms=args.speed_ms)
    return 0


def snapshot(args: argparse.Namespace) -> int:
    write_snapshot(args.input, args.output, args.time, cell=args.cell)
    if args.verbose:
        print(f"wrote {args.output}")
    return 0


def simulate(args: argparse.Namespace) -> int:
    scenario = load_scenario(args.input)
    result = run_event_simulation(
        scenario=scenario,
        lacam_binary=args.binary,
        map_dir=args.map_dir,
        max_rounds=args.max_rounds,
        max_steps=args.max_steps,
        work_dir=args.work_dir,
        keep_round_files=args.keep_round_files,
        timeout_s=args.timeout,
        time_limit=args.time_limit,
        anytime=not args.no_anytime,
        verbose=args.verbose,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    if args.verbose:
        print(f"wrote {args.output}")
        print(f"stop_reason={result['stop_reason']} final_time={result['final_time']} delivered={result['total_delivered_ore']}")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sim = sub.add_parser("simulate", help="run event-driven lifelong ore simulation")
    sim.add_argument("--input", type=Path, required=True)
    sim.add_argument("--binary", type=Path, default=Path("build/tapf_benchmark"))
    sim.add_argument("--map-dir", type=Path, default=Path("."))
    sim.add_argument("--output", type=Path, required=True)
    sim.add_argument("--work-dir", type=Path, required=True)
    sim.add_argument("--max-rounds", type=int, default=100)
    sim.add_argument("--max-steps", type=int, default=2000)
    sim.add_argument("--timeout", type=float, default=30.0)
    sim.add_argument("--time-limit", type=float, default=10.0)
    sim.add_argument("--keep-round-files", action="store_true")
    sim.add_argument("--no-anytime", action="store_true")
    sim.add_argument("--verbose", action="store_true")
    sim.set_defaults(func=simulate)

    rep = sub.add_parser("replay", help="open an interactive Tk replay window for a simulation JSON")
    rep.add_argument("--input", type=Path, required=True)
    rep.add_argument("--cell", type=int, default=26)
    rep.add_argument("--speed-ms", type=int, default=150)
    rep.set_defaults(func=replay)

    snap = sub.add_parser("snapshot", help="render one replay frame to a PNG file")
    snap.add_argument("--input", type=Path, required=True)
    snap.add_argument("--output", type=Path, required=True)
    snap.add_argument("--time", type=int, default=0)
    snap.add_argument("--cell", type=int, default=28)
    snap.add_argument("--verbose", action="store_true")
    snap.set_defaults(func=snapshot)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
