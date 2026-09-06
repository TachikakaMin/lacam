#!/usr/bin/env python3
"""Generate a warehouse testcase proposal page and certified sample cases.

This page is deliberately separate from the protected release benchmark.  Its
four sample instances are real DD YAML files with validator-checked solvability
certificates, while the browser displays only Planner runs.  The remaining
rows describe the proposed follow-up benchmark matrix.
"""

import argparse
import hashlib
import json
import os
import subprocess
import sys
from collections import deque
from pathlib import Path
from typing import Dict, List, Sequence, Set, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ddbench.instance import Cell, Instance, Target
from ddbench.validator import (
    apply_joint_action,
    initial_state,
    plan_cost,
    validate_plan,
)
from generate_web_viz import parse_plan
from generate_warehouse_block_sample import block_geometry
from generate_warehouse_factorial_suite import (
    factorial_case_specs,
    generate_suite as generate_factorial_suite,
)


HERE = Path(__file__).resolve().parent
PLANNER_TIME_LIMIT_SEC = 10
PLANNER_SEED = 0
PLANNER_MODE = "lacam"


def proposal_cases() -> List[Dict]:
    """Return the complete paired 432-case factorial matrix."""
    rows = []
    for spec in factorial_case_specs():
        block_size = spec["block_size"]
        period = block_size + spec["aisle_width"]
        cells_per_block = block_size * block_size
        block_rows = spec["height"] // period
        block_cols = spec["width"] // period
        block_count = block_rows * block_cols
        pool_size = spec["goal_pool_size"]
        eligible_union = (
            spec["targets"]
            if spec["goal_mode"] == "singleton"
            else pool_size
        )
        rows.append(
            {
                **spec,
                "axis": "配对全因子",
                "block_rows": block_rows,
                "block_cols": block_cols,
                "block_count": block_count,
                "cells_per_block": cells_per_block,
                "actual_density": (
                    spec["shelves_per_block"] / cells_per_block
                ),
                "storage_fraction": (block_size / period) ** 2,
                "total_shelves": (
                    block_count * spec["shelves_per_block"]
                ),
                "vacancies_per_block": (
                    cells_per_block - spec["shelves_per_block"]
                ),
                "profile": spec["task_profile"],
                "goals_per_target": (
                    1
                    if spec["goal_mode"] == "singleton"
                    else pool_size
                ),
                "goal_pool_scope": (
                    "per_target_fixed"
                    if spec["goal_mode"] == "singleton"
                    else "global_uniform_slack"
                ),
                "eligible_goal_union_size": eligible_union,
                "covering_matching_margin": (
                    eligible_union - spec["targets"]
                ),
                "core_goal_count": spec["targets"] // 2,
                "core_goal_policy": "goal_depth_at_least_1",
                "shared_layout_group": spec["pairing_key"],
                "robot_start_policy": "nested_prefix",
                "seed_controls": (
                    "paired_tasks_goals_shelf_priority_robot_order"
                ),
                "certification": "validator_checked",
                "yaml": "factorial_suite/instances/{}.yaml".format(
                    spec["id"]
                ),
                "note": (
                    "{} / {} / {}；同组仅改变密度和 Agent 数".format(
                        spec["density_level"],
                        spec["agent_level"],
                        spec["task_profile"],
                    )
                ),
            }
        )
    return rows


def _sample_specs() -> List[Dict]:
    remote_out = (
        [(2, 16), (1, 16), (0, 16)]
        + [(0, c) for c in range(15, 3, -1)]
        + [(r, 4) for r in range(1, 11)]
        + [(10, 3)]
    )
    remote_back = (
        [(9, 4)]
        + [(r, 4) for r in range(8, -1, -1)]
        + [(0, c) for c in range(5, 17)]
        + [(r, 16) for r in range(1, 4)]
        + [(3, 17)]
    )
    return [
        {
            "name": "warehouse_cert_h8w8_b3_a1_s4of9_r2_t4_mixed_seed0",
            "title": "样例 A：小地图混合任务",
            "description": "2 个 block 内任务 + 2 个相邻跨区任务。",
            "height": 8,
            "width": 8,
            "block_size": 3,
            "aisle_width": 1,
            "shelves_per_block": 4,
            "robots": [(0, 0), (4, 4)],
            "tasks": [
                {"id": "b0", "start": (1, 1), "goal": (1, 2),
                 "loaded_path": [(1, 2)]},
                {"id": "b1", "start": (7, 7), "goal": (7, 6),
                 "loaded_path": [(7, 6)]},
                {"id": "b2", "start": (2, 3), "goal": (2, 5),
                 "loaded_path": [(2, 4), (2, 5)]},
                {"id": "b3", "start": (3, 5), "goal": (3, 3),
                 "loaded_path": [(3, 4), (3, 3)]},
            ],
        },
        {
            "name": "warehouse_cert_h10w10_b4_a1_s12of16_r4_t4_core_seed0",
            "title": "样例 B：75% 密度 + block 内层 goal",
            "description": "展示高密度、相邻跨区和一个需要进入 block 内层的 goal。",
            "height": 10,
            "width": 10,
            "block_size": 4,
            "aisle_width": 1,
            "shelves_per_block": 12,
            "robots": [(0, 0), (5, 5), (0, 5), (5, 0)],
            "tasks": [
                {"id": "b0", "start": (1, 1), "goal": (1, 2),
                 "loaded_path": [(1, 2)]},
                {"id": "b2", "start": (3, 4), "goal": (3, 6),
                 "loaded_path": [(3, 5), (3, 6)]},
                {"id": "b3", "start": (4, 6), "goal": (4, 4),
                 "loaded_path": [(4, 5), (4, 4)]},
                {"id": "b1", "start": (9, 9), "goal": (8, 8),
                 "loaded_path": [(8, 9), (8, 8)]},
            ],
        },
        {
            "name": "warehouse_cert_h12w20_b3_a1_s7of9_r6_t6_remote_seed0",
            "title": "样例 C：矩形地图 + 远距离跨区任务",
            "description": "同时包含同区、相邻跨区，以及一对方向相反的远距离任务。",
            "height": 12,
            "width": 20,
            "block_size": 3,
            "aisle_width": 1,
            "shelves_per_block": 7,
            "robots": [
                (0, 0),
                (4, 8),
                (8, 8),
                (4, 12),
                (8, 12),
                (4, 0),
            ],
            "tasks": [
                {"id": "b0", "start": (1, 1), "goal": (1, 2),
                 "loaded_path": [(1, 2)]},
                {"id": "b2", "start": (6, 7), "goal": (6, 9),
                 "loaded_path": [(6, 8), (6, 9)]},
                {"id": "b3", "start": (7, 9), "goal": (7, 7),
                 "loaded_path": [(7, 8), (7, 7)]},
                {"id": "b4", "start": (2, 17), "goal": (10, 3),
                 "loaded_path": remote_out},
                {"id": "b5", "start": (9, 3), "goal": (3, 17),
                 "loaded_path": remote_back},
                {"id": "b1", "start": (11, 19), "goal": (10, 18),
                 "loaded_path": [(10, 19), (10, 18)]},
            ],
        },
        {
            "name": "warehouse_cert_h8w8_b3_a1_s4of9_r3_t3_pool4_seed0",
            "title": "样例 D：共享 Goal Pool（3 件货选 4 个位置）",
            "description": (
                "3 件目标货架共享同一个 pool，每件货有 4 个 goal；"
                "最终需占用 3 个互异位置，并保留 1 个 spare。"
            ),
            "height": 8,
            "width": 8,
            "block_size": 3,
            "aisle_width": 1,
            "shelves_per_block": 4,
            "goal_mode": "shared_pool_slack",
            "goal_pool": [(2, 5), (3, 3), (5, 5), (7, 7)],
            "robots": [(0, 0), (4, 0), (0, 4)],
            "joint_witness": [
                [
                    ("move", (0, 1)),
                    ("move", (4, 1)),
                    ("move", (0, 5)),
                ],
                [
                    ("move", (0, 2)),
                    ("move", (4, 2)),
                    ("move", (1, 5)),
                ],
                [
                    ("move", (0, 3)),
                    ("move", (4, 3)),
                    ("move", (2, 5)),
                ],
                [
                    ("move", (1, 3)),
                    ("move", (5, 3)),
                    ("move", (3, 5)),
                ],
                [("move", (2, 3)), ("lift",), ("lift",)],
                [
                    ("lift",),
                    ("move", (4, 3)),
                    ("move", (4, 5)),
                ],
                [
                    ("move", (2, 4)),
                    ("move", (3, 3)),
                    ("move", (5, 5)),
                ],
                [("move", (2, 5)), ("drop",), ("drop",)],
                [("drop",), ("wait",), ("wait",)],
            ],
            "tasks": [
                {
                    "id": "b0",
                    "start": (2, 3),
                    "witness_assignment": (2, 5),
                    "loaded_path": [(2, 4), (2, 5)],
                },
                {
                    "id": "b1",
                    "start": (3, 5),
                    "witness_assignment": (5, 5),
                    "loaded_path": [(4, 5), (5, 5)],
                },
                {
                    "id": "b2",
                    "start": (5, 3),
                    "witness_assignment": (3, 3),
                    "loaded_path": [(4, 3), (3, 3)],
                },
            ],
        },
    ]


def _cell_block(
    cell: Cell,
    block_size: int,
    aisle_width: int,
    block_cols: int,
) -> int:
    period = block_size + aisle_width
    row, col = cell
    block_row = row // period
    block_col = col // period
    return block_row * block_cols + block_col


def _relation(
    start: Cell,
    goal: Cell,
    block_size: int,
    aisle_width: int,
    block_cols: int,
) -> str:
    source = _cell_block(start, block_size, aisle_width, block_cols)
    destination = _cell_block(goal, block_size, aisle_width, block_cols)
    sr, sc = divmod(source, block_cols)
    dr, dc = divmod(destination, block_cols)
    distance = abs(sr - dr) + abs(sc - dc)
    if distance == 0:
        return "intra"
    if distance == 1:
        return "adjacent"
    return "remote"


def _goal_depth(goal: Cell, block_size: int, aisle_width: int) -> int:
    period = block_size + aisle_width
    local_r = (goal[0] % period) - aisle_width
    local_c = (goal[1] % period) - aisle_width
    return min(
        local_r,
        local_c,
        block_size - 1 - local_r,
        block_size - 1 - local_c,
    )


def _bfs_robot_path(
    height: int,
    width: int,
    start: Cell,
    goal: Cell,
    blocked: Set[Cell],
) -> List[Cell]:
    if start == goal:
        return []
    queue = deque([start])
    parent = {start: None}
    while queue:
        current = queue.popleft()
        for dr, dc in ((1, 0), (0, 1), (-1, 0), (0, -1)):
            nxt = (current[0] + dr, current[1] + dc)
            if not (0 <= nxt[0] < height and 0 <= nxt[1] < width):
                continue
            if nxt in blocked or nxt in parent:
                continue
            parent[nxt] = current
            if nxt == goal:
                queue.clear()
                break
            queue.append(nxt)
    if goal not in parent:
        raise ValueError("robot cannot reach {}".format(goal))
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = parent[current]
    path.reverse()
    return path


def _joint(active_action: Tuple, robots: int) -> List[Tuple]:
    return [active_action] + [("wait",)] * (robots - 1)


def _joint_witness_from_spec(
    spec: Dict, robot_count: int
) -> List[List[Tuple]]:
    raw_plan = spec.get("joint_witness")
    if raw_plan is None:
        return []
    if not raw_plan:
        raise ValueError("{}: empty joint witness".format(spec["name"]))
    plan = []
    for timestep, raw_joint in enumerate(raw_plan):
        if len(raw_joint) != robot_count:
            raise ValueError(
                "{}: joint witness t={} has {} actions for {} robots".format(
                    spec["name"], timestep, len(raw_joint), robot_count
                )
            )
        joint = []
        for robot, raw_action in enumerate(raw_joint):
            if not raw_action:
                raise ValueError(
                    "{}: empty action at t={} robot={}".format(
                        spec["name"], timestep, robot
                    )
                )
            kind = raw_action[0]
            if kind == "move" and len(raw_action) == 2:
                joint.append(("move", tuple(raw_action[1])))
            elif kind in ("wait", "lift", "drop") and len(raw_action) == 1:
                joint.append((kind,))
            else:
                raise ValueError(
                    "{}: malformed action {} at t={} robot={}".format(
                        spec["name"], raw_action, timestep, robot
                    )
                )
        plan.append(joint)
    return plan


def _witness_activity(
    ins: Instance, plan: Sequence[Sequence[Tuple]]
) -> Tuple[List[Dict[str, int]], List[List[str]]]:
    activity = [
        {"move": 0, "lift": 0, "drop": 0, "wait": 0}
        for _ in ins.robots
    ]
    target_ids = [[] for _ in ins.robots]
    state = initial_state(ins)
    for joint in plan:
        carried = state.carried_shelves()
        target_at = {
            cell: target_id
            for target_id, cell in state.target_pos
            if target_id not in carried
        }
        for robot, action in enumerate(joint):
            activity[robot][action[0]] += 1
            if action[0] == "lift":
                target_id = target_at.get(state.robots[robot])
                if target_id is not None:
                    target_ids[robot].append(target_id)
        state = apply_joint_action(ins, state, joint)
    return activity, target_ids


def _witness_assignment(task: Dict) -> Cell:
    key = "witness_assignment" if "witness_assignment" in task else "goal"
    return tuple(task[key])


def _materialize_sample(spec: Dict) -> Tuple[Instance, List[List[Tuple]], Dict]:
    height = spec["height"]
    width = spec["width"]
    block_size = spec["block_size"]
    aisle_width = spec["aisle_width"]
    blocks = block_geometry(height, width, block_size, aisle_width)
    storage = {cell for block in blocks for cell in block}
    tasks = spec["tasks"]

    starts = {tuple(task["start"]) for task in tasks}
    assignments = {_witness_assignment(task) for task in tasks}
    goal_pool = (
        sorted({tuple(cell) for cell in spec["goal_pool"]})
        if spec.get("goal_pool")
        else None
    )
    if goal_pool is not None and len(goal_pool) != len(spec["goal_pool"]):
        raise ValueError("{}: duplicate goal-pool cell".format(spec["name"]))
    if len(starts) != len(tasks) or len(assignments) != len(tasks):
        raise ValueError("{}: duplicate target endpoint".format(spec["name"]))
    if not starts <= storage or not assignments <= storage:
        raise ValueError("{}: target endpoint outside storage".format(spec["name"]))
    if goal_pool is not None:
        if not set(goal_pool) <= storage:
            raise ValueError("{}: goal pool outside storage".format(spec["name"]))
        if not assignments <= set(goal_pool):
            raise ValueError(
                "{}: witness assignment outside goal pool".format(spec["name"])
            )

    reserved_empty = set(assignments)
    if goal_pool is not None:
        reserved_empty.update(goal_pool)
    for task in tasks:
        previous = tuple(task["start"])
        for step in task["loaded_path"]:
            step = tuple(step)
            if abs(previous[0] - step[0]) + abs(previous[1] - step[1]) != 1:
                raise ValueError(
                    "{}: non-adjacent loaded path".format(spec["name"])
                )
            if step in storage and step != tuple(task["start"]):
                reserved_empty.add(step)
            previous = step
        if previous != _witness_assignment(task):
            raise ValueError(
                "{}: loaded path misses witness assignment".format(
                    spec["name"]
                )
            )
    if starts & reserved_empty:
        raise ValueError("{}: start reserved as empty".format(spec["name"]))

    shelves = set()
    expected = spec["shelves_per_block"]
    for block in blocks:
        block_set = set(block)
        required = sorted(block_set & starts)
        forbidden = block_set & reserved_empty
        candidates = sorted(block_set - set(required) - forbidden)
        need = expected - len(required)
        if need < 0 or need > len(candidates):
            raise ValueError(
                "{}: cannot realize uniform block density".format(spec["name"])
            )
        shelves.update(required)
        shelves.update(candidates[:need])

    targets = [
        Target(
            id=task["id"],
            start=tuple(task["start"]),
            goal=(
                goal_pool[0]
                if goal_pool is not None
                else _witness_assignment(task)
            ),
            goals=list(goal_pool) if goal_pool is not None else None,
        )
        for task in tasks
    ]
    ins = Instance(
        grid=[[False] * width for _ in range(height)],
        robots=[tuple(cell) for cell in spec["robots"]],
        shelves=sorted(shelves),
        targets=targets,
        name=spec["name"],
        goal_pool=list(goal_pool) if goal_pool is not None else None,
        storage_cells=storage,
    )
    errors = ins.validate_static()
    if errors:
        raise ValueError("{}: {}".format(spec["name"], errors))

    plan = _joint_witness_from_spec(spec, len(ins.robots))
    witness_kind = "explicit_joint"
    if not plan:
        witness_kind = "sequential_single_robot"
        parked = set(ins.robots[1:])
        active = ins.robots[0]
        for task in tasks:
            pickup = tuple(task["start"])
            for cell in _bfs_robot_path(
                height, width, active, pickup, parked
            ):
                plan.append(_joint(("move", cell), len(ins.robots)))
                active = cell
            plan.append(_joint(("lift",), len(ins.robots)))
            for cell in task["loaded_path"]:
                cell = tuple(cell)
                if cell in parked:
                    raise ValueError(
                        "{}: parked robot blocks witness".format(spec["name"])
                    )
                plan.append(_joint(("move", cell), len(ins.robots)))
                active = cell
            plan.append(_joint(("drop",), len(ins.robots)))

    ok, plan_errors, final_state = validate_plan(ins, plan)
    if not ok:
        raise ValueError(
            "{}: witness invalid: {}".format(spec["name"], plan_errors)
        )
    declared_assignments = {
        task["id"]: _witness_assignment(task) for task in tasks
    }
    final_assignments = dict(final_state.target_pos)
    if set(declared_assignments) != set(final_assignments):
        raise ValueError("{}: witness target IDs differ".format(spec["name"]))
    for target_id, declared in declared_assignments.items():
        actual = final_assignments[target_id]
        if actual != declared:
            raise ValueError(
                "{}: target {} witness ends at {}, declared {}".format(
                    spec["name"], target_id, actual, declared
                )
            )
    robot_activity, robot_target_ids = _witness_activity(ins, plan)
    active_robot_indices = [
        robot
        for robot, target_ids in enumerate(robot_target_ids)
        if target_ids
    ]

    block_cols = width // (block_size + aisle_width)
    relations = {"intra": 0, "adjacent": 0, "remote": 0}
    task_rows = []
    targets_by_id = {target.id: target for target in ins.targets}
    for task in tasks:
        assignment = final_assignments[task["id"]]
        kind = _relation(
            tuple(task["start"]),
            assignment,
            block_size,
            aisle_width,
            block_cols,
        )
        relations[kind] += 1
        depth = _goal_depth(
            assignment, block_size, aisle_width
        )
        target = targets_by_id[task["id"]]
        task_rows.append(
            {
                "id": task["id"],
                "start": list(task["start"]),
                "witness_assignment": list(assignment),
                "eligible_goals": [
                    list(cell) for cell in target.eligible_goals()
                ],
                "relation": kind,
                "goal_depth": depth,
            }
        )
    eligible_goal_union = sorted(
        {
            tuple(goal)
            for target in ins.targets
            for goal in target.eligible_goals()
        }
    )
    metadata = {
        "blocks": blocks,
        "relations": relations,
        "relation_basis": "witness_assignment",
        "core_goals": sum(row["goal_depth"] > 0 for row in task_rows),
        "task_rows": task_rows,
        "goal_mode": spec.get("goal_mode", "fixed_singleton"),
        "goal_pool": list(goal_pool) if goal_pool is not None else [],
        "goal_pool_size": len(goal_pool) if goal_pool is not None else 0,
        "goals_per_target": len(ins.targets[0].eligible_goals()),
        "eligible_goal_union": eligible_goal_union,
        "witness_assignments": sorted(final_assignments.values()),
        "spare_goals": sorted(
            set(eligible_goal_union) - set(final_assignments.values())
        ),
        "witness_kind": witness_kind,
        "robot_activity": robot_activity,
        "robot_target_ids": robot_target_ids,
        "active_robot_indices": active_robot_indices,
        "active_robot_count": len(active_robot_indices),
    }
    return ins, plan, metadata


def _action_token(action: Tuple) -> str:
    if action[0] == "wait":
        return "w"
    if action[0] == "move":
        return "m {} {}".format(action[1][0], action[1][1])
    if action[0] == "lift":
        return "l"
    if action[0] == "drop":
        return "d"
    raise ValueError("unknown action {}".format(action))


def _plan_text(plan: Sequence[Sequence[Tuple]]) -> str:
    return "\n".join(
        ";".join(_action_token(action) for action in joint)
        for joint in plan
    ) + "\n"


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _generate_animation(
    yaml_path: Path,
    plan_path: Path,
    animation_path: Path,
    title: str,
) -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(HERE / "generate_web_viz.py"),
            str(yaml_path),
            str(plan_path),
            str(animation_path),
            "--title",
            title,
        ],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
    )
    if not animation_path.is_file():
        raise RuntimeError(
            "animation generation failed for {}: {}".format(
                plan_path, result.stdout
            )
        )


def _planner_display_metadata(
    spec: Dict,
    ins: Instance,
    assignments: Dict[str, Cell],
) -> Dict:
    targets_by_id = {target.id: target for target in ins.targets}
    task_ids = {task["id"] for task in spec["tasks"]}
    if set(assignments) != task_ids:
        raise RuntimeError(
            "planner assignment IDs differ for {}".format(spec["name"])
        )

    assigned_cells = set(assignments.values())
    if len(assigned_cells) != len(assignments):
        raise RuntimeError(
            "planner assignments collide for {}".format(spec["name"])
        )

    block_cols = ins.width // (
        spec["block_size"] + spec["aisle_width"]
    )
    relations = {"intra": 0, "adjacent": 0, "remote": 0}
    task_rows = []
    for task in spec["tasks"]:
        target_id = task["id"]
        start = tuple(task["start"])
        assignment = assignments[target_id]
        goals = set(targets_by_id[target_id].eligible_goals())
        if assignment not in goals:
            raise RuntimeError(
                "planner assignment outside eligible goals for {} {}".format(
                    spec["name"], target_id
                )
            )
        if len(goals) == 1 and assignment != next(iter(goals)):
            raise RuntimeError(
                "planner missed fixed goal for {} {}".format(
                    spec["name"], target_id
                )
            )
        relation = _relation(
            start,
            assignment,
            spec["block_size"],
            spec["aisle_width"],
            block_cols,
        )
        depth = _goal_depth(
            assignment,
            spec["block_size"],
            spec["aisle_width"],
        )
        relations[relation] += 1
        task_rows.append(
            {
                "id": target_id,
                "start": list(start),
                "assignment": list(assignment),
                "relation": relation,
                "goal_depth": depth,
            }
        )

    eligible_goal_union = {
        goal
        for target in ins.targets
        for goal in target.eligible_goals()
    }
    return {
        "tasks": task_rows,
        "relations": relations,
        "core_goals": sum(
            task["goal_depth"] > 0 for task in task_rows
        ),
        "spare_goals": sorted(
            eligible_goal_union - assigned_cells
        ),
    }


def _planner_run(
    spec: Dict,
    ins: Instance,
    yaml_path: Path,
    output_dir: Path,
    planner_binary: Path,
    binary_sha256: str,
) -> Dict:
    planner_dir = output_dir / "planner_runs"
    planner_dir.mkdir(parents=True, exist_ok=True)
    plan_rel = "planner_runs/{}.planner.plan".format(spec["name"])
    animation_rel = "planner_runs/{}.html".format(spec["name"])
    plan_path = output_dir / plan_rel
    animation_path = output_dir / animation_rel

    result = subprocess.run(
        [
            str(planner_binary),
            str(yaml_path),
            str(PLANNER_TIME_LIMIT_SEC),
            str(plan_path),
            str(PLANNER_SEED),
        ],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "planner failed for {} with exit {}:\n{}".format(
                spec["name"],
                result.returncode,
                result.stderr or result.stdout,
            )
        )
    if not plan_path.is_file():
        raise RuntimeError(
            "planner did not create plan for {}".format(spec["name"])
        )

    plan = parse_plan(plan_path)
    ok, errors, final_state = validate_plan(ins, plan)
    if not ok:
        raise RuntimeError(
            "planner plan invalid for {}: {}".format(
                spec["name"], errors
            )
        )
    robot_activity, robot_target_ids = _witness_activity(ins, plan)
    active_robot_indices = [
        robot
        for robot, target_ids in enumerate(robot_target_ids)
        if target_ids
    ]
    assignments = {
        target_id: cell
        for target_id, cell in final_state.target_pos
    }
    display = _planner_display_metadata(spec, ins, assignments)
    metrics = plan_cost(ins, plan)
    _generate_animation(
        yaml_path,
        plan_path,
        animation_path,
        "{} · Planner seed {} · makespan {} · weighted SOC {}".format(
            spec["title"],
            PLANNER_SEED,
            metrics["executed_makespan"],
            metrics["weighted_soc"],
        ),
    )
    return {
        "seed": PLANNER_SEED,
        "time_limit_sec": PLANNER_TIME_LIMIT_SEC,
        "mode": PLANNER_MODE,
        "binary_sha256": binary_sha256,
        "plan_sha256": _sha256_file(plan_path),
        "plan": plan_rel,
        "animation": animation_rel,
        "steps": len(plan),
        "metrics": metrics,
        "robot_activity": robot_activity,
        "robot_target_ids": robot_target_ids,
        "active_robot_indices": active_robot_indices,
        "active_robot_count": len(active_robot_indices),
        "assignments": {
            target_id: list(cell)
            for target_id, cell in assignments.items()
        },
        "tasks": display["tasks"],
        "relations": display["relations"],
        "core_goals": display["core_goals"],
        "spare_goals": [
            list(cell) for cell in display["spare_goals"]
        ],
    }


def _write_sample_yaml(
    spec: Dict,
    ins: Instance,
    plan: Sequence[Sequence[Tuple]],
    metadata: Dict,
    path: Path,
) -> str:
    plan_sha = hashlib.sha256(_plan_text(plan).encode("utf-8")).hexdigest()
    lines = [
        "# Witness-certified warehouse testcase proposal sample",
        "# The witness is a proof artifact; benchmark solvers must not read it.",
        "name: {}".format(ins.name),
        "map: |",
    ]
    lines.extend("  " + "." * ins.width for _ in range(ins.height))
    lines.append("storage_map: |")
    lines.extend(
        "  "
        + "".join(
            "S" if (r, c) in ins.storage_cells else "."
            for c in range(ins.width)
        )
        for r in range(ins.height)
    )
    cells_per_block = spec["block_size"] ** 2
    density = spec["shelves_per_block"] / cells_per_block
    lines.extend(
        [
            "warehouse_layout:",
            "  block_size: {}".format(spec["block_size"]),
            "  aisle_width: {}".format(spec["aisle_width"]),
            "  requested_density: {:.8f}".format(density),
            "  actual_density: {:.8f}".format(density),
            "  shelves_per_block: {}".format(spec["shelves_per_block"]),
            "  corridor_policy: hard_no_drop",
            "  block_origins:",
        ]
    )
    lines.extend(
        "    - [{}, {}]".format(block[0][0], block[0][1])
        for block in metadata["blocks"]
    )
    lines.extend(
        [
            "task_profile:",
            "  goal_mode: {}".format(metadata["goal_mode"]),
            "  goals_per_target: {}".format(
                metadata["goals_per_target"]
            ),
            "  goal_pool_size: {}".format(metadata["goal_pool_size"]),
            "  relation_basis: {}".format(metadata["relation_basis"]),
            "  witness_kind: {}".format(metadata["witness_kind"]),
            "  active_robot_count: {}".format(
                metadata["active_robot_count"]
            ),
            "  active_robot_indices: [{}]".format(
                ", ".join(
                    str(robot)
                    for robot in metadata["active_robot_indices"]
                )
            ),
            "  relation_counts:",
            "    intra: {}".format(metadata["relations"]["intra"]),
            "    adjacent: {}".format(metadata["relations"]["adjacent"]),
            "    remote: {}".format(metadata["relations"]["remote"]),
            "  core_goals: {}".format(metadata["core_goals"]),
            "  witness_guarantee: validator_checked",
            "  witness_steps: {}".format(len(plan)),
            "  witness_sha256: {}".format(plan_sha),
            "robots:",
        ]
    )
    lines.extend("  - [{}, {}]".format(*cell) for cell in ins.robots)
    lines.append("shelves:")
    lines.extend("  - [{}, {}]".format(*cell) for cell in ins.shelves)
    if ins.goal_pool is not None:
        lines.append("goal_pool:")
        lines.extend(
            "  - [{}, {}]".format(*cell) for cell in ins.goal_pool
        )
    lines.append("targets:")
    for target in ins.targets:
        lines.extend(
            [
                "  - id: {}".format(target.id),
                "    start: [{}, {}]".format(*target.start),
            ]
        )
        if (
            ins.goal_pool is not None
            and list(target.eligible_goals()) == list(ins.goal_pool)
        ):
            lines.append("    goals: pool")
        elif target.goals is not None:
            lines.append("    goals:")
            lines.extend(
                "      - [{}, {}]".format(*goal)
                for goal in target.eligible_goals()
            )
        else:
            lines.append("    goal: [{}, {}]".format(*target.goal))
    lines.extend(
        [
            "flags:",
            "  remove_on_complete: false",
            "  robots_return_to_rest: false",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")
    return plan_sha


def _sample_record(
    spec: Dict,
    ins: Instance,
    plan: Sequence[Sequence[Tuple]],
    metadata: Dict,
    yaml_rel: str,
    witness_rel: str,
    animation_rel: str,
    plan_sha: str,
    planner: Dict = None,
) -> Dict:
    return {
        "name": spec["name"],
        "title": spec["title"],
        "description": spec["description"],
        "height": ins.height,
        "width": ins.width,
        "block_size": spec["block_size"],
        "aisle_width": spec["aisle_width"],
        "shelves_per_block": spec["shelves_per_block"],
        "cells_per_block": spec["block_size"] ** 2,
        "density": spec["shelves_per_block"] / (spec["block_size"] ** 2),
        "blocks": len(metadata["blocks"]),
        "robots": len(ins.robots),
        "shelves": len(ins.shelves),
        "targets": len(ins.targets),
        "witness_steps": len(plan),
        "witness_sha256": plan_sha,
        "certification": "validator_checked",
        "relations": metadata["relations"],
        "relation_basis": metadata["relation_basis"],
        "core_goals": metadata["core_goals"],
        "goal_mode": metadata["goal_mode"],
        "goal_pool_size": metadata["goal_pool_size"],
        "goals_per_target": metadata["goals_per_target"],
        "eligible_goal_union": [
            list(cell) for cell in metadata["eligible_goal_union"]
        ],
        "witness_assignments": [
            list(cell) for cell in metadata["witness_assignments"]
        ],
        "spare_goals": [
            list(cell) for cell in metadata["spare_goals"]
        ],
        "witness_kind": metadata["witness_kind"],
        "robot_activity": metadata["robot_activity"],
        "robot_target_ids": metadata["robot_target_ids"],
        "active_robot_indices": metadata["active_robot_indices"],
        "active_robot_count": metadata["active_robot_count"],
        "tasks": metadata["task_rows"],
        "storage": [list(cell) for cell in sorted(ins.storage_cells)],
        "shelf_cells": [list(cell) for cell in ins.shelves],
        "robot_cells": [list(cell) for cell in ins.robots],
        "yaml": yaml_rel,
        "witness": witness_rel,
        "animation": animation_rel,
        "planner": planner,
    }


def _presentation_record(record: Dict) -> Dict:
    """Return browser data without solvability-certificate behavior."""
    visible_keys = (
        "name",
        "title",
        "description",
        "height",
        "width",
        "block_size",
        "aisle_width",
        "shelves_per_block",
        "cells_per_block",
        "robots",
        "targets",
        "goal_mode",
        "goal_pool_size",
        "goals_per_target",
        "eligible_goal_union",
        "storage",
        "shelf_cells",
        "robot_cells",
        "yaml",
        "planner",
    )
    presentation = {
        key: record[key]
        for key in visible_keys
    }
    presentation["target_starts"] = [
        {
            "id": task["id"],
            "start": task["start"],
        }
        for task in record["tasks"]
    ]
    return presentation


PAGE_TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Warehouse testcase Planner 实跑提案</title>
<style>
  :root {
    color-scheme: dark;
    --bg:#08111f; --panel:#111d30; --panel2:#0c1728; --line:#263a55;
    --text:#edf5ff; --muted:#9eb0c7; --blue:#60a5fa; --green:#34d399;
    --amber:#fbbf24; --red:#fb7185; --aisle:#20344a; --storage:#d8c49a;
    --shelf:#7c4a21; --target:#f59e0b; --purple:#a78bfa;
  }
  * { box-sizing:border-box; }
  body {
    margin:0; background:
      radial-gradient(circle at 12% -10%, #14345d 0, transparent 32rem),
      var(--bg); color:var(--text);
    font-family:-apple-system,BlinkMacSystemFont,"Segoe UI","PingFang SC",
      "Microsoft YaHei",sans-serif;
  }
  main { max-width:1440px; margin:auto; padding:28px 24px 52px; }
  h1 { margin:0 0 10px; font-size:clamp(25px,3vw,38px); }
  h2 { margin:0 0 12px; font-size:22px; }
  h3 { margin:0 0 8px; font-size:17px; }
  p { line-height:1.65; }
  .lead { max-width:980px; color:#c5d4e6; margin:0 0 18px; }
  .badges,.legend,.links,.filters { display:flex; flex-wrap:wrap; gap:9px; }
  .badge,.chip {
    border:1px solid var(--line); background:var(--panel2);
    border-radius:999px; padding:7px 11px; font-size:13px; color:#c9d8ea;
  }
  .badge.good { color:#a7f3d0; border-color:#216b58; }
  section { margin-top:28px; }
  .sample-card,.table-wrap,.player {
    background:linear-gradient(145deg,var(--panel),#0c1727);
    border:1px solid var(--line); border-radius:14px;
  }
  .note {
    border-left:3px solid var(--amber); background:#2c2417;
    color:#fde9a9; padding:12px 14px; border-radius:5px; line-height:1.6;
  }
  .sample-grid {
    display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:14px;
  }
  .sample-card { padding:15px; min-width:0; }
  .sample-card canvas {
    display:block; width:100%; aspect-ratio:1.35; background:#07111e;
    border-radius:9px; margin:10px 0 12px;
  }
  .sample-card p { color:var(--muted); font-size:14px; margin:6px 0 10px; }
  .links { align-items:center; margin-top:12px; }
  a,button { font:inherit; }
  a { color:#7dd3fc; text-decoration:none; }
  a:hover { text-decoration:underline; }
  button {
    border:0; border-radius:8px; background:#2563eb; color:white;
    padding:8px 12px; cursor:pointer;
  }
  button:hover { background:#1d4ed8; }
  .legend { color:var(--muted); font-size:12px; margin-top:8px; }
  .sw { width:12px; height:12px; border-radius:3px; display:inline-block; }
  .player { padding:14px; }
  .player-head {
    display:flex; justify-content:space-between; align-items:center;
    gap:12px; margin-bottom:10px;
  }
  iframe {
    width:100%; height:690px; border:1px solid var(--line);
    background:#1e1e24; border-radius:10px;
  }
  .filters { margin:12px 0; }
  select,input {
    background:#0a1525; color:var(--text); border:1px solid var(--line);
    border-radius:8px; padding:9px 11px;
  }
  input { min-width:260px; }
  .table-wrap { overflow:auto; }
  table { width:100%; border-collapse:collapse; min-width:1080px; }
  th,td { padding:10px 11px; border-bottom:1px solid #20324a; text-align:left; }
  th {
    position:sticky; top:0; background:#132239; color:#cbdced;
    font-size:12px; letter-spacing:.02em;
  }
  td { font-size:13px; color:#c6d4e6; }
  tr:hover td { background:#122139; }
  .profile { font-weight:700; color:#e5effc; }
  .footer { color:var(--muted); font-size:13px; margin-top:24px; }
  @media (max-width:1000px) {
    .sample-grid { grid-template-columns:1fr; }
    iframe { height:620px; }
  }
  @media (max-width:600px) {
    main { padding:20px 13px 40px; }
  }
</style>
</head>
<body>
<main>
  <h1>Warehouse testcase 扩展提案</h1>
  <p class="lead">本页分成两部分：下方 4 个原有已认证样例继续展示当前 Planner 的真实运行结果；另外 432 个随机配对 testcase 已生成真实 YAML，并全部通过构造可解性认证。它们组成 6 个地图族 × 3 个密度 × 4 个 Agent 数 × 3 种任务分布 × 2 种 Goal 模式。</p>
  <div class="badges">
    <span class="badge good">✓ 4 个已认证样例全部通过</span>
    <span class="badge good">✓ 432 个随机配对 testcase</span>
    <span class="badge">全部通过构造可解性认证</span>
    <span class="badge">通道只允许运输，禁止 DROP</span>
    <span class="badge">初始每个 block 密度完全一致</span>
  </div>

  <section>
    <h2>术语说明</h2>
    <p class="lead"><b>Planner assignment：Planner 实跑最终为每件货选择的终点。</b> eligible goal：某件货允许作为最终位置的全部格子；storage cell 是允许货架落下的货架位；通道只能让载货机器人经过。intra：起点和终点位于同一个货架区；adjacent：终点位于相邻货架区；remote：至少跨越两个货架区；内层 goal：目标格不在 block 边缘。</p>
  </section>

  <section>
    <h2>可解性保障</h2>
    <p class="note">4 个原有样例和 432 个随机 testcase 都已确认理论可解；后台可解性证书已由 validator 验证，不是 Planner 输出，也不会在本页播放，也不会作为 solver 输入。随机 testcase 不会根据 Planner 成功率、makespan 或 SOC 筛选 seed。</p>
  </section>

  <section>
    <h2>4 个已认证样例</h2>
    <div class="legend">
      <span><i class="sw" style="background:var(--aisle)"></i> 通道</span>
      <span><i class="sw" style="background:var(--storage)"></i> storage</span>
      <span><i class="sw" style="background:var(--shelf)"></i> 匿名货架</span>
      <span><i class="sw" style="background:var(--target)"></i> target start</span>
      <span><i class="sw" style="border:2px dashed var(--green)"></i> eligible goal</span>
      <span><i class="sw" style="background:var(--purple)"></i> Planner assignment</span>
      <span><i class="sw" style="background:var(--blue);border-radius:50%"></i> robot</span>
    </div>
    <p class="lead"><b>本页只展示 Planner 实跑。</b> Planner 使用 mode=lacam、seed=0、10 秒上限，输出 plan 会再次由 authoritative validator 检查。active agent 数量不是正确性目标，而是“实际 lift/carry 过 target 的 agent 数”；它可能因优化目标和调度选择而少于总 agent 数。</p>
    <p class="lead">静态图上的虚线框表示全部 eligible goals，<b>静态紫色标记表示 Planner assignment。</b> 起点到 Planner assignment 的连线，不是 Planner 的实际运动轨迹；同区/邻区/远区按 Planner assignment 统计。</p>
    <div class="sample-grid" id="sampleGrid"></div>
  </section>

  <section class="player">
    <div class="player-head">
      <div><h2 id="playerTitle">Planner 实跑未生成</h2><span id="playerMeta"></span></div>
      <a id="openAnimation" href="" target="_blank" hidden>单独打开动画</a>
    </div>
    <iframe id="sampleFrame" title="planner animation"></iframe>
  </section>

  <section>
    <h2>432 个随机配对 testcase</h2>
    <p class="lead">每个地图族都完整覆盖 3 个密度 × 4 个 Agent 数，并分别覆盖 local、mixed、cross-heavy 三种任务分布及单 goal、共享 goal pool 两种模式。Block 密度使用“每块货架数 / block cell 数”的实际值。</p>
    <p class="lead"><b>密度对照：</b>低、中、高密度使用嵌套货架布局；低密度货架是中密度的子集，中密度又是高密度的子集。新增的只会是随机匿名货架，target 起点和 goal 不变。</p>
    <p class="lead"><b>Agent 数量对照：</b>Agent 起点使用嵌套前缀；较少 Agent 的起点列表严格是较多 Agent 起点列表的前缀。</p>
    <p class="lead"><b>配对随机：</b>同一配对组保持 target 起点、goal 定义和随机基准不变，只交叉改变密度和 Agent 数。不会根据 Planner 成功率、makespan 或 SOC 筛选 seed。</p>
    <div class="filters">
      <select id="axisFilter"><option value="">全部测试轴</option></select>
      <select id="profileFilter"><option value="">全部任务分布</option></select>
      <select id="goalModeFilter"><option value="">全部 Goal 模式</option></select>
      <input id="caseSearch" type="search" placeholder="搜索 id、尺寸或说明">
      <span class="chip" id="visibleCount"></span>
    </div>
    <div class="table-wrap">
      <table>
        <thead><tr>
          <th>ID</th><th>测试轴</th><th>地图</th><th>Block</th>
          <th>Blocks</th><th>每块密度</th><th>Storage 占图</th>
          <th>Agents</th><th>Tasks</th><th>Goal 模式</th>
          <th>每件货候选数</th><th>候选并集</th>
          <th>同区/邻区/远区</th>
          <th>内层 goal</th><th>认证状态</th><th>Seed</th>
          <th>对照关系</th><th>说明</th>
        </tr></thead>
        <tbody id="proposalBody"></tbody>
      </table>
    </div>
  </section>

  <p class="footer">432 个新 testcase 已成为 protected full benchmark；开发期 quick suite 仍固定为原有 77 cases，full benchmark 需在实现完成并通过独立 review 后运行。</p>
</main>
<script>
const PROPOSALS = __PROPOSALS__;
const SAMPLES = __SAMPLES__;
const key = p => `${p[0]},${p[1]}`;
const relationColor = {intra:"#34d399", adjacent:"#fbbf24", remote:"#fb7185"};
const relationLabel = {intra:"同一货架区", adjacent:"相邻货架区", remote:"远距离跨区"};
const goalModeLabel = {
  singleton:"固定单 goal",
  shared_pool:"共享 slack pool",
  fixed_singleton:"固定单 goal",
  shared_pool_exact:"共享 exact pool",
  shared_pool_slack:"共享 slack pool",
  shared_pool_hotspot:"共享 hotspot pool"
};

function drawArrow(ctx, x1, y1, x2, y2, color, dashed) {
  const angle = Math.atan2(y2-y1, x2-x1);
  ctx.save();
  ctx.strokeStyle=color; ctx.fillStyle=color; ctx.lineWidth=2.2;
  ctx.setLineDash(dashed ? [5,4] : []);
  ctx.beginPath(); ctx.moveTo(x1,y1); ctx.lineTo(x2,y2); ctx.stroke();
  ctx.setLineDash([]);
  ctx.beginPath();
  ctx.moveTo(x2,y2);
  ctx.lineTo(x2-8*Math.cos(angle-.48),y2-8*Math.sin(angle-.48));
  ctx.lineTo(x2-8*Math.cos(angle+.48),y2-8*Math.sin(angle+.48));
  ctx.closePath(); ctx.fill(); ctx.restore();
}

function drawSample(canvas, s) {
  const cssWidth = 520;
  const cssHeight = Math.max(250, cssWidth * s.height / s.width);
  const dpr = window.devicePixelRatio || 1;
  canvas.width=cssWidth*dpr; canvas.height=cssHeight*dpr;
  canvas.style.aspectRatio=`${s.width}/${s.height}`;
  const ctx=canvas.getContext("2d"); ctx.scale(dpr,dpr);
  const cw=cssWidth/s.width, ch=cssHeight/s.height;
  const storage=new Set(s.storage.map(key));
  const shelves=new Set(s.shelf_cells.map(key));
  const starts=new Map(s.target_starts.map(t=>[key(t.start),t]));
  for(let r=0;r<s.height;r++) for(let c=0;c<s.width;c++) {
    ctx.fillStyle=storage.has(`${r},${c}`)?"#d8c49a":"#20344a";
    ctx.fillRect(c*cw,r*ch,cw+.25,ch+.25);
    ctx.strokeStyle="rgba(8,17,31,.22)";
    ctx.strokeRect(c*cw,r*ch,cw,ch);
  }
  for(const p of s.shelf_cells) {
    const t=starts.get(key(p));
    ctx.fillStyle=t?"#f59e0b":"#7c4a21";
    ctx.fillRect(p[1]*cw+cw*.22,p[0]*ch+ch*.22,cw*.56,ch*.56);
  }
  for(const p of s.eligible_goal_union) {
    ctx.save();
    ctx.strokeStyle="#22c55e"; ctx.lineWidth=2;
    ctx.setLineDash([5,3]);
    ctx.strokeRect(p[1]*cw+2,p[0]*ch+2,cw-4,ch-4);
    ctx.restore();
  }
  if(s.planner) {
    for(const t of s.planner.tasks) {
      const color=relationColor[t.relation];
      const x1=(t.start[1]+.5)*cw, y1=(t.start[0]+.5)*ch;
      const x2=(t.assignment[1]+.5)*cw;
      const y2=(t.assignment[0]+.5)*ch;
      drawArrow(ctx,x1,y1,x2,y2,color,t.relation==="remote");
    }
    for(const p of Object.values(s.planner.assignments)) {
      ctx.save();
      ctx.strokeStyle="#a78bfa"; ctx.lineWidth=3;
      ctx.strokeRect(
        p[1]*cw+Math.max(4,cw*.18),
        p[0]*ch+Math.max(4,ch*.18),
        cw-2*Math.max(4,cw*.18),
        ch-2*Math.max(4,ch*.18)
      );
      ctx.restore();
    }
  }
  for(const p of s.robot_cells) {
    ctx.fillStyle="#3b82f6"; ctx.beginPath();
    ctx.arc((p[1]+.5)*cw,(p[0]+.5)*ch,Math.max(2.5,Math.min(cw,ch)*.25),0,Math.PI*2);
    ctx.fill();
  }
}

const assignmentText = assignments => Object.entries(assignments)
  .map(([target,cell])=>`${target}→(${cell[0]},${cell[1]})`).join(" · ");

function showPlannerUnavailable() {
  const frame=document.getElementById("sampleFrame");
  const link=document.getElementById("openAnimation");
  frame.removeAttribute("src");
  frame.title="planner animation unavailable";
  document.getElementById("playerTitle").textContent="Planner 实跑未生成";
  document.getElementById("playerMeta").textContent=
    "请使用当前 dd_benchmark 重新生成本页。";
  link.hidden=true;
  link.removeAttribute("href");
}

function openSample(index, shouldScroll=true) {
  const s=SAMPLES[index];
  if(!s.planner) {
    showPlannerUnavailable();
    return;
  }
  const link=document.getElementById("openAnimation");
  document.getElementById("sampleFrame").src=s.planner.animation;
  document.getElementById("sampleFrame").title=`${s.title} · Planner 实跑`;
  document.getElementById("playerTitle").textContent=`${s.title} · Planner 实跑`;
  document.getElementById("playerMeta").textContent=
    `${s.height}×${s.width} · ${s.planner.active_robot_count}/${s.robots} active agents · ${s.targets} tasks · ${goalModeLabel[s.goal_mode]} · ${s.goals_per_target} goals/target · makespan ${s.planner.metrics.executed_makespan} · weighted SOC ${s.planner.metrics.weighted_soc} · seed ${s.planner.seed} · ${s.planner.time_limit_sec}s`;
  link.href=s.planner.animation;
  link.hidden=false;
  if(shouldScroll)
    document.querySelector(".player").scrollIntoView({behavior:"smooth",block:"start"});
}

document.getElementById("sampleGrid").innerHTML=SAMPLES.map((s,i)=>`
  <article class="sample-card">
    <h3>${s.title}</h3>
    <p>${s.description}</p>
    <canvas data-sample="${i}"></canvas>
    <div class="badges">
      <span class="chip">${s.height}×${s.width}</span>
      <span class="chip">B=${s.block_size}</span>
      <span class="chip">${s.shelves_per_block}/${s.cells_per_block}</span>
      <span class="chip">${s.robots} agents</span>
      ${s.planner
        ?`<span class="chip">Planner ${s.planner.active_robot_count}/${s.robots} active agents</span>
          <span class="chip">makespan ${s.planner.metrics.executed_makespan}</span>
          <span class="chip">weighted SOC ${s.planner.metrics.weighted_soc}</span>`
        :`<span class="chip">Planner 实跑未生成</span>`}
      <span class="chip">${s.targets} tasks</span>
      <span class="chip">${goalModeLabel[s.goal_mode]}</span>
      <span class="chip">${s.goals_per_target===1
        ?"每件货 1 个固定 goal"
        :`每件货有 ${s.goals_per_target} 个 goal`}</span>
    </div>
    ${s.planner?`<div class="badges">
      ${Object.entries(s.planner.relations).filter(x=>x[1]).map(([k,v])=>
        `<span class="chip" style="color:${relationColor[k]}">${relationLabel[k]} ${v}</span>`).join("")}
      <span class="chip">内层目标 ${s.planner.core_goals}</span>
      ${s.planner.spare_goals.length
        ?`<span class="chip">未占用 pool goal ${s.planner.spare_goals.length}</span>`
        :""}
    </div>`:""}
    ${s.planner
      ?`<p><b>Planner assignment:</b> ${assignmentText(s.planner.assignments)}</p>`
      :""}
    <div class="links">
      <a href="${s.yaml}">YAML</a>
      __PLANNER_ACTIONS__
    </div>
  </article>`).join("");
document.querySelectorAll("canvas[data-sample]").forEach(c=>
  drawSample(c,SAMPLES[Number(c.dataset.sample)]));

const axis=document.getElementById("axisFilter");
const profile=document.getElementById("profileFilter");
const goalMode=document.getElementById("goalModeFilter");
for(const value of [...new Set(PROPOSALS.map(r=>r.axis))].sort())
  axis.insertAdjacentHTML("beforeend",`<option>${value}</option>`);
for(const value of [...new Set(PROPOSALS.map(r=>r.profile))].sort())
  profile.insertAdjacentHTML("beforeend",`<option>${value}</option>`);
for(const value of [...new Set(PROPOSALS.map(r=>r.goal_mode))].sort())
  goalMode.insertAdjacentHTML(
    "beforeend",
    `<option value="${value}">${goalModeLabel[value]}</option>`
  );

function renderRows() {
  const q=document.getElementById("caseSearch").value.trim().toLowerCase();
  const rows=PROPOSALS.filter(r=>
    (!axis.value||r.axis===axis.value) &&
    (!profile.value||r.profile===profile.value) &&
    (!goalMode.value||r.goal_mode===goalMode.value) &&
    (!q||JSON.stringify(r).toLowerCase().includes(q)));
  document.getElementById("visibleCount").textContent=`显示 ${rows.length}/432`;
  document.getElementById("proposalBody").innerHTML=rows.map(r=>{
    const comparison=`${r.pairing_key}；密度/Agent 完整交叉`;
    return `
    <tr><td><a href="${r.yaml}"><b>${r.id}</b></a></td><td>${r.axis}</td>
    <td>${r.height}×${r.width}</td><td>${r.block_size}×${r.block_size}</td>
    <td>${r.block_rows}×${r.block_cols}=${r.block_count}</td>
    <td>${r.shelves_per_block}/${r.cells_per_block} (${(r.actual_density*100).toFixed(1)}%)</td>
    <td>${(r.storage_fraction*100).toFixed(1)}%</td>
    <td>${r.robots}</td><td>${r.targets}</td>
    <td>${goalModeLabel[r.goal_mode]}</td>
    <td>${r.goals_per_target}</td>
    <td>${r.eligible_goal_union_size}${r.goal_pool_size
      ?`（pool ${r.goal_pool_size}）`
      :"（固定配对）"}</td>
    <td class="profile">${r.relation_counts.intra}/${r.relation_counts.adjacent}/${r.relation_counts.remote} · ${r.profile}</td>
    <td>${r.core_goal_count}</td><td>validator 已认证</td>
    <td>${r.seed}</td><td>${comparison}</td><td>${r.note}</td></tr>`;
  }).join("");
}
axis.onchange=renderRows; profile.onchange=renderRows;
goalMode.onchange=renderRows;
document.getElementById("caseSearch").oninput=renderRows;
renderRows();
__INITIAL_PLAYER__
</script>
</body>
</html>
"""


def _render_page(proposals: Sequence[Dict], samples: Sequence[Dict]) -> str:
    presentation_samples = [
        _presentation_record(record)
        for record in samples
    ]
    first_planner = next(
        (
            index
            for index, sample in enumerate(presentation_samples)
            if sample["planner"] is not None
        ),
        None,
    )
    if first_planner is None:
        planner_actions = ""
        initial_player = "showPlannerUnavailable();"
    else:
        planner_actions = r"""${s.planner
        ?`<button onclick="openSample(${i})">播放 Planner 实跑</button>
          <a href="${s.planner.plan}">Planner plan</a>
          <a href="${s.planner.animation}" target="_blank">Planner 动画</a>`
        :""}"""
        initial_player = "openSample({},false);".format(first_planner)
    return (
        PAGE_TEMPLATE.replace(
            "__PROPOSALS__",
            json.dumps(proposals, ensure_ascii=False, separators=(",", ":")),
        )
        .replace(
            "__SAMPLES__",
            json.dumps(
                presentation_samples,
                ensure_ascii=False,
                separators=(",", ":"),
            ),
        )
        .replace("__PLANNER_ACTIONS__", planner_actions)
        .replace("__INITIAL_PLAYER__", initial_player)
    )


def generate_site(
    output_dir: Path, planner_binary: Path = None
) -> Dict:
    output_dir = Path(output_dir)
    if planner_binary is not None:
        planner_binary = Path(planner_binary).resolve()
        if not planner_binary.is_file():
            raise ValueError(
                "planner binary does not exist: {}".format(
                    planner_binary
                )
            )
        if not os.access(planner_binary, os.X_OK):
            raise ValueError(
                "planner binary is not executable: {}".format(
                    planner_binary
                )
            )
        binary_sha256 = _sha256_file(planner_binary)
    else:
        binary_sha256 = ""
    samples_dir = output_dir / "samples"
    animations_dir = output_dir / "animations"
    samples_dir.mkdir(parents=True, exist_ok=True)
    animations_dir.mkdir(parents=True, exist_ok=True)

    records = []
    for spec in _sample_specs():
        ins, plan, metadata = _materialize_sample(spec)
        yaml_rel = "samples/{}.yaml".format(spec["name"])
        witness_rel = "samples/{}.witness.plan".format(spec["name"])
        animation_rel = "animations/{}.html".format(spec["name"])
        yaml_path = output_dir / yaml_rel
        witness_path = output_dir / witness_rel
        animation_path = output_dir / animation_rel

        witness_text = _plan_text(plan)
        witness_path.write_text(witness_text, encoding="utf-8")
        plan_sha = _write_sample_yaml(
            spec, ins, plan, metadata, yaml_path
        )
        if plan_sha != hashlib.sha256(
            witness_text.encode("utf-8")
        ).hexdigest():
            raise AssertionError("witness hash mismatch")

        _generate_animation(
            yaml_path,
            witness_path,
            animation_path,
            "{} · witness {} steps".format(
                spec["title"], len(plan)
            ),
        )
        planner = (
            _planner_run(
                spec,
                ins,
                yaml_path,
                output_dir,
                planner_binary,
                binary_sha256,
            )
            if planner_binary is not None
            else None
        )
        records.append(
            _sample_record(
                spec,
                ins,
                plan,
                metadata,
                yaml_rel,
                witness_rel,
                animation_rel,
                plan_sha,
                planner,
            )
        )

    factorial_summary = generate_factorial_suite(
        output_dir / "factorial_suite"
    )
    proposals = proposal_cases()
    (output_dir / "proposal.json").write_text(
        json.dumps(proposals, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (output_dir / "samples.json").write_text(
        json.dumps(records, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (output_dir / "index.html").write_text(
        _render_page(proposals, records), encoding="utf-8"
    )
    return {
        "proposal_count": len(proposals),
        "factorial_case_count": factorial_summary["case_count"],
        "sample_count": len(records),
        "planner_run_count": sum(
            record["planner"] is not None for record in records
        ),
        "samples": records,
        "index": "index.html",
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=HERE / "viz_web" / "warehouse_case_proposal",
    )
    parser.add_argument(
        "--planner-binary",
        type=Path,
        default=HERE.parent / "build" / "dd_benchmark",
        help="dd_benchmark binary used for seed-0 10-second runs",
    )
    parser.add_argument(
        "--no-planner-runs",
        action="store_true",
        help="generate witness-only output explicitly",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    planner_binary = None if args.no_planner_runs else args.planner_binary
    if planner_binary is not None:
        if not planner_binary.is_file():
            print(
                "error: planner binary does not exist: {}".format(
                    planner_binary
                ),
                file=sys.stderr,
            )
            return 2
        if not os.access(planner_binary, os.X_OK):
            print(
                "error: planner binary is not executable: {}".format(
                    planner_binary
                ),
                file=sys.stderr,
            )
            return 2
    summary = generate_site(
        args.output_dir, planner_binary=planner_binary
    )
    print("index={}".format(args.output_dir / summary["index"]))
    print(
        "proposal_cases={}, certified_samples={}, planner_runs={}".format(
            summary["proposal_count"],
            summary["sample_count"],
            summary["planner_run_count"],
        )
    )
    for record in summary["samples"]:
        planner_summary = ""
        if record["planner"] is not None:
            planner_summary = (
                ", planner_makespan={}, planner_weighted_soc={}, "
                "planner_active={}/{}"
            ).format(
                record["planner"]["metrics"]["executed_makespan"],
                record["planner"]["metrics"]["weighted_soc"],
                record["planner"]["active_robot_count"],
                record["robots"],
            )
        print(
            "{}: witness_steps={}, animation={}{}".format(
                record["name"],
                record["witness_steps"],
                args.output_dir / record["animation"],
                planner_summary,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
