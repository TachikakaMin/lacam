#!/usr/bin/env python3
"""Generate the paired 432-case random warehouse factorial suite.

Each random scenario fixes map geometry, target starts, goal definitions, and
an ordered robot-start list.  It then expands to every density/agent
combination.  Shelf layouts are nested by density and robot starts are nested
prefixes, so measured changes can be attributed to the controlled factors.

Feasibility is established by a validator-checked sequential certificate.  A
certificate is never passed to the benchmark solver and is not linked from
the visualization page.
"""

import argparse
import hashlib
import json
import math
import random
import sys
from collections import deque
from pathlib import Path
from typing import Dict, List, Sequence, Set, Tuple

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ddbench.instance import Cell, Instance, Target, dump_map_str
from ddbench.validator import validate_plan
from generate_warehouse_block_sample import block_geometry


HERE = Path(__file__).resolve().parent

DENSITY_LEVELS = ("low", "medium", "high")
AGENT_LEVELS = ("scarce", "baseline", "equal", "surplus")
TASK_PROFILES = ("local", "mixed", "cross_heavy")
GOAL_MODES = ("singleton", "shared_pool")

MAP_FAMILIES = {
    "g1": {
        "height": 8,
        "width": 8,
        "block_size": 3,
        "targets": 4,
        "densities": {"low": 2, "medium": 5, "high": 7},
        "agents": {"scarce": 2, "baseline": 3, "equal": 4, "surplus": 6},
    },
    "g2": {
        "height": 12,
        "width": 12,
        "block_size": 3,
        "targets": 6,
        "densities": {"low": 2, "medium": 5, "high": 7},
        "agents": {"scarce": 3, "baseline": 4, "equal": 6, "surplus": 9},
    },
    "g3": {
        "height": 12,
        "width": 20,
        "block_size": 3,
        "targets": 8,
        "densities": {"low": 2, "medium": 5, "high": 7},
        "agents": {"scarce": 4, "baseline": 6, "equal": 8, "surplus": 12},
    },
    "g4": {
        "height": 20,
        "width": 20,
        "block_size": 4,
        "targets": 12,
        "densities": {"low": 4, "medium": 8, "high": 12},
        "agents": {"scarce": 6, "baseline": 8, "equal": 12, "surplus": 18},
    },
    "g5": {
        "height": 24,
        "width": 24,
        "block_size": 5,
        "targets": 16,
        "densities": {"low": 6, "medium": 13, "high": 19},
        "agents": {"scarce": 8, "baseline": 12, "equal": 16, "surplus": 24},
    },
    "g6": {
        "height": 30,
        "width": 30,
        "block_size": 9,
        "targets": 18,
        "densities": {"low": 20, "medium": 41, "high": 61},
        "agents": {"scarce": 9, "baseline": 12, "equal": 18, "surplus": 27},
    },
}

PROFILE_WEIGHTS = {
    "local": {"intra": 0.70, "adjacent": 0.30, "remote": 0.0},
    "mixed": {"intra": 0.50, "adjacent": 1.0 / 3.0, "remote": 1.0 / 6.0},
    "cross_heavy": {
        "intra": 1.0 / 6.0,
        "adjacent": 0.50,
        "remote": 1.0 / 3.0,
    },
}


def _stable_seed(*parts) -> int:
    digest = hashlib.sha256(
        "::".join(str(part) for part in parts).encode("utf-8")
    ).digest()
    return int.from_bytes(digest[:8], "big") & 0x7FFFFFFF


def _profile_counts(profile: str, targets: int) -> Dict[str, int]:
    weights = PROFILE_WEIGHTS[profile]
    names = ("intra", "adjacent", "remote")
    raw = {name: weights[name] * targets for name in names}
    counts = {name: int(math.floor(raw[name])) for name in names}
    remaining = targets - sum(counts.values())
    order = sorted(
        names,
        key=lambda name: (raw[name] - counts[name], -names.index(name)),
        reverse=True,
    )
    for name in order[:remaining]:
        counts[name] += 1
    return counts


def factorial_case_specs(seed: int = 0) -> List[Dict]:
    rows = []
    for family_id, family in MAP_FAMILIES.items():
        relation_counts_by_profile = {
            profile: _profile_counts(profile, family["targets"])
            for profile in TASK_PROFILES
        }
        for density_level in DENSITY_LEVELS:
            for agent_level in AGENT_LEVELS:
                for profile in TASK_PROFILES:
                    for goal_mode in GOAL_MODES:
                        pairing_seed = _stable_seed(
                            "warehouse-factorial",
                            seed,
                            family_id,
                            profile,
                            goal_mode,
                        )
                        row_id = (
                            "{}_d{}_a{}_p{}_g{}_seed{}".format(
                                family_id,
                                density_level,
                                agent_level,
                                profile,
                                goal_mode,
                                seed,
                            )
                        )
                        pool_size = (
                            int(math.ceil(1.5 * family["targets"]))
                            if goal_mode == "shared_pool"
                            else 0
                        )
                        rows.append(
                            {
                                "id": row_id,
                                "map_family": family_id,
                                "height": family["height"],
                                "width": family["width"],
                                "block_size": family["block_size"],
                                "aisle_width": 1,
                                "targets": family["targets"],
                                "density_level": density_level,
                                "shelves_per_block": family["densities"][
                                    density_level
                                ],
                                "agent_level": agent_level,
                                "robots": family["agents"][agent_level],
                                "task_profile": profile,
                                "relation_counts": dict(
                                    relation_counts_by_profile[profile]
                                ),
                                "goal_mode": goal_mode,
                                "goal_pool_size": pool_size,
                                "pairing_key": "{}__{}__{}__seed{}".format(
                                    family_id, profile, goal_mode, seed
                                ),
                                "pairing_seed": pairing_seed,
                                "seed": seed,
                                "layout_policy": "nested_density_prefix",
                                "robot_start_policy": "nested_agent_prefix",
                                "certification": "validator_checked",
                            }
                        )
    if len(rows) != 432 or len({row["id"] for row in rows}) != 432:
        raise AssertionError("factorial matrix must contain 432 unique cases")
    return rows


def _block_index(
    cell: Cell, block_size: int, block_cols: int
) -> int:
    period = block_size + 1
    return (cell[0] // period) * block_cols + (cell[1] // period)


def _block_distance(a: int, b: int, block_cols: int) -> int:
    ar, ac = divmod(a, block_cols)
    br, bc = divmod(b, block_cols)
    return abs(ar - br) + abs(ac - bc)


def _relation_for_blocks(
    source: int, destination: int, block_cols: int
) -> str:
    distance = _block_distance(source, destination, block_cols)
    if distance == 0:
        return "intra"
    if distance == 1:
        return "adjacent"
    return "remote"


def _goal_depth(cell: Cell, block_size: int) -> int:
    period = block_size + 1
    local_r = cell[0] % period - 1
    local_c = cell[1] % period - 1
    return min(
        local_r,
        local_c,
        block_size - 1 - local_r,
        block_size - 1 - local_c,
    )


def _gateway_options(
    block: Sequence[Cell],
    storage: Set[Cell],
    height: int,
    width: int,
    block_size: int,
) -> List[Dict]:
    block_set = set(block)
    options = []
    for cell in block:
        for outward in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            corridor = (cell[0] + outward[0], cell[1] + outward[1])
            if not (
                0 <= corridor[0] < height
                and 0 <= corridor[1] < width
                and corridor not in storage
            ):
                continue
            inner = (cell[0] - outward[0], cell[1] - outward[1])
            options.append(
                {
                    "slot": cell,
                    "corridor": corridor,
                    "inner": (
                        inner
                        if inner in block_set
                        and _goal_depth(inner, block_size) >= 1
                        else None
                    ),
                }
            )
    return options


def _bfs_path(
    start: Cell,
    goal: Cell,
    allowed: Set[Cell],
    blocked: Set[Cell],
    rng: random.Random,
) -> List[Cell]:
    if start == goal:
        return []
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    rng.shuffle(directions)
    queue = deque([start])
    parent = {start: None}
    while queue:
        current = queue.popleft()
        for dr, dc in directions:
            nxt = (current[0] + dr, current[1] + dc)
            if (
                nxt not in allowed
                or nxt in blocked
                or nxt in parent
            ):
                continue
            parent[nxt] = current
            if nxt == goal:
                queue.clear()
                break
            queue.append(nxt)
    if goal not in parent:
        raise ValueError("no path from {} to {}".format(start, goal))
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = parent[current]
    path.reverse()
    return path


def _scenario_blueprint(
    family_id: str,
    profile: str,
    goal_mode: str,
    seed: int,
) -> Dict:
    family = MAP_FAMILIES[family_id]
    height = family["height"]
    width = family["width"]
    block_size = family["block_size"]
    blocks = block_geometry(height, width, block_size, 1)
    storage = {cell for block in blocks for cell in block}
    corridor = {
        (r, c)
        for r in range(height)
        for c in range(width)
        if (r, c) not in storage
    }
    block_cols = width // (block_size + 1)
    low_shelves = family["densities"]["low"]
    high_vacancies = block_size * block_size - family["densities"]["high"]
    relation_counts = _profile_counts(profile, family["targets"])
    scenario_seed = _stable_seed(
        "warehouse-factorial", seed, family_id, profile, goal_mode
    )

    for attempt in range(500):
        rng = random.Random(scenario_seed + attempt * 104729)
        gateway_options = [
            _gateway_options(
                block, storage, height, width, block_size
            )
            for block in blocks
        ]
        relation_sequence = [
            relation
            for relation, count in relation_counts.items()
            for _ in range(count)
        ]
        rng.shuffle(relation_sequence)
        inner_sequence = [True] * (family["targets"] // 2)
        inner_sequence += [False] * (
            family["targets"] - len(inner_sequence)
        )
        rng.shuffle(inner_sequence)

        starts = set()
        reserved_empty = set()
        source_counts = [0] * len(blocks)
        empty_counts = [0] * len(blocks)
        tasks = []
        failed = False
        for target_index, (relation, inner_goal) in enumerate(
            zip(relation_sequence, inner_sequence)
        ):
            choices = []
            for source_block in range(len(blocks)):
                if source_counts[source_block] >= low_shelves:
                    continue
                for destination_block in range(len(blocks)):
                    if (
                        _relation_for_blocks(
                            source_block, destination_block, block_cols
                        )
                        != relation
                    ):
                        continue
                    for start_option in gateway_options[source_block]:
                        start = start_option["slot"]
                        if start in starts or start in reserved_empty:
                            continue
                        for goal_option in gateway_options[destination_block]:
                            if inner_goal:
                                if goal_option["inner"] is None:
                                    continue
                                storage_path = [
                                    goal_option["slot"],
                                    goal_option["inner"],
                                ]
                            else:
                                storage_path = [goal_option["slot"]]
                            if (
                                len(set(storage_path)) != len(storage_path)
                                or set(storage_path) & starts
                                or set(storage_path) & reserved_empty
                                or start in storage_path
                            ):
                                continue
                            if (
                                empty_counts[destination_block]
                                + len(storage_path)
                                > high_vacancies
                            ):
                                continue
                            choices.append(
                                (
                                    source_block,
                                    destination_block,
                                    start_option,
                                    goal_option,
                                    storage_path,
                                )
                            )
            if not choices:
                failed = True
                break
            (
                source_block,
                destination_block,
                start_option,
                goal_option,
                storage_path,
            ) = rng.choice(choices)
            corridor_path = _bfs_path(
                start_option["corridor"],
                goal_option["corridor"],
                corridor,
                set(),
                rng,
            )
            loaded_path = [start_option["corridor"]]
            loaded_path.extend(corridor_path)
            loaded_path.extend(storage_path)
            assignment = storage_path[-1]
            tasks.append(
                {
                    "id": "b{}".format(target_index),
                    "start": start_option["slot"],
                    "assignment": assignment,
                    "loaded_path": loaded_path,
                    "relation": relation,
                    "goal_depth": _goal_depth(assignment, block_size),
                }
            )
            starts.add(start_option["slot"])
            reserved_empty.update(storage_path)
            source_counts[source_block] += 1
            empty_counts[destination_block] += len(storage_path)
        if failed:
            continue

        assignments = [task["assignment"] for task in tasks]
        goal_pool = []
        if goal_mode == "shared_pool":
            goal_pool = list(assignments)
            required_pool_size = int(
                math.ceil(1.5 * family["targets"])
            )
            candidates = list(storage - starts - reserved_empty)
            rng.shuffle(candidates)
            for cell in candidates:
                if len(goal_pool) >= required_pool_size:
                    break
                block_index = _block_index(
                    cell, block_size, block_cols
                )
                if empty_counts[block_index] >= high_vacancies:
                    continue
                goal_pool.append(cell)
                reserved_empty.add(cell)
                empty_counts[block_index] += 1
            if len(goal_pool) != required_pool_size:
                continue
            goal_pool = sorted(goal_pool)

        shelf_order_by_block = []
        for block in blocks:
            required = sorted(set(block) & starts)
            forbidden = set(block) & reserved_empty
            candidates = sorted(
                set(block) - set(required) - forbidden
            )
            rng.shuffle(candidates)
            if (
                len(required) > low_shelves
                or len(required) + len(candidates)
                < family["densities"]["high"]
            ):
                failed = True
                break
            shelf_order_by_block.append(required + candidates)
        if failed:
            continue

        loaded_corridor = {
            cell
            for task in tasks
            for cell in task["loaded_path"]
            if cell in corridor
        }
        robot_candidates = sorted(corridor - loaded_corridor)
        max_robots = max(family["agents"].values())
        if len(robot_candidates) < max_robots:
            continue
        robot_order = rng.sample(robot_candidates, max_robots)

        return {
            "family_id": family_id,
            "height": height,
            "width": width,
            "block_size": block_size,
            "blocks": blocks,
            "storage": storage,
            "corridor": corridor,
            "tasks": tasks,
            "goal_mode": goal_mode,
            "goal_pool": goal_pool,
            "relation_counts": relation_counts,
            "shelf_order_by_block": shelf_order_by_block,
            "robot_order": robot_order,
            "scenario_seed": scenario_seed,
            "generation_attempt": attempt,
        }
    raise RuntimeError(
        "cannot generate scenario {} {} {} seed {}".format(
            family_id, profile, goal_mode, seed
        )
    )


def _joint(action: Tuple, robots: int) -> List[Tuple]:
    return [action] + [("wait",)] * (robots - 1)


def _certificate(
    ins: Instance,
    tasks: Sequence[Dict],
) -> List[List[Tuple]]:
    all_cells = {
        (r, c)
        for r in range(ins.height)
        for c in range(ins.width)
        if not ins.grid[r][c]
    }
    parked = set(ins.robots[1:])
    active = ins.robots[0]
    rng = random.Random(_stable_seed(ins.name, "certificate"))
    plan = []
    for task in tasks:
        pickup = tuple(task["start"])
        for cell in _bfs_path(
            active, pickup, all_cells, parked, rng
        ):
            plan.append(_joint(("move", cell), len(ins.robots)))
            active = cell
        plan.append(_joint(("lift",), len(ins.robots)))
        for cell in task["loaded_path"]:
            cell = tuple(cell)
            if cell in parked:
                raise ValueError(
                    "{}: parked robot blocks loaded path".format(ins.name)
                )
            plan.append(_joint(("move", cell), len(ins.robots)))
            active = cell
        plan.append(_joint(("drop",), len(ins.robots)))
    ok, errors, _ = validate_plan(ins, plan)
    if not ok:
        raise ValueError(
            "{}: invalid feasibility certificate: {}".format(
                ins.name, errors
            )
        )
    return plan


def _materialize_variant(blueprint: Dict, spec: Dict) -> Dict:
    shelves = []
    for order in blueprint["shelf_order_by_block"]:
        shelves.extend(order[: spec["shelves_per_block"]])
    shelves = sorted(shelves)
    robots = list(blueprint["robot_order"][: spec["robots"]])
    pool = list(blueprint["goal_pool"]) or None
    targets = []
    for task in blueprint["tasks"]:
        if pool is None:
            targets.append(
                Target(
                    id=task["id"],
                    start=tuple(task["start"]),
                    goal=tuple(task["assignment"]),
                )
            )
        else:
            targets.append(
                Target(
                    id=task["id"],
                    start=tuple(task["start"]),
                    goal=tuple(pool[0]),
                    goals=list(pool),
                )
            )
    ins = Instance(
        grid=[
            [False] * blueprint["width"]
            for _ in range(blueprint["height"])
        ],
        robots=robots,
        shelves=shelves,
        targets=targets,
        name=spec["id"],
        goal_pool=pool,
        storage_cells=set(blueprint["storage"]),
    )
    errors = ins.validate_static()
    if errors:
        raise ValueError("{}: {}".format(spec["id"], errors))
    certificate = _certificate(ins, blueprint["tasks"])
    metadata = {
        "pairing_key": spec["pairing_key"],
        "pairing_seed": spec["pairing_seed"],
        "generation_attempt": blueprint["generation_attempt"],
        "relation_counts": dict(blueprint["relation_counts"]),
        "core_goal_count": sum(
            task["goal_depth"] > 0 for task in blueprint["tasks"]
        ),
        "certificate_steps": len(certificate),
        "certificate_sha256": hashlib.sha256(
            _plan_text(certificate).encode("utf-8")
        ).hexdigest(),
        "assignments": {
            task["id"]: list(task["assignment"])
            for task in blueprint["tasks"]
        },
    }
    return {
        "spec": spec,
        "instance": ins,
        "certificate": certificate,
        "metadata": metadata,
        "tasks": blueprint["tasks"],
        "blocks": blueprint["blocks"],
    }


def materialize_pairing_group(
    family_id: str,
    profile: str,
    goal_mode: str,
    seed: int = 0,
) -> List[Dict]:
    if family_id not in MAP_FAMILIES:
        raise ValueError("unknown map family {}".format(family_id))
    if profile not in TASK_PROFILES:
        raise ValueError("unknown task profile {}".format(profile))
    if goal_mode not in GOAL_MODES:
        raise ValueError("unknown goal mode {}".format(goal_mode))
    blueprint = _scenario_blueprint(
        family_id, profile, goal_mode, seed
    )
    specs = [
        row
        for row in factorial_case_specs(seed)
        if row["map_family"] == family_id
        and row["task_profile"] == profile
        and row["goal_mode"] == goal_mode
    ]
    return [_materialize_variant(blueprint, spec) for spec in specs]


def _action_token(action: Tuple) -> str:
    if action[0] == "wait":
        return "w"
    if action[0] == "move":
        return "m {} {}".format(*action[1])
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


def _variant_yaml(variant: Dict) -> str:
    ins = variant["instance"]
    spec = variant["spec"]
    metadata = variant["metadata"]
    data = {
        "name": ins.name,
        "map": dump_map_str(ins.grid) + "\n",
        "storage_map": "\n".join(
            "".join(
                "S" if (r, c) in ins.storage_cells else "."
                for c in range(ins.width)
            )
            for r in range(ins.height)
        )
        + "\n",
        "warehouse_layout": {
            "map_family": spec["map_family"],
            "block_size": spec["block_size"],
            "aisle_width": 1,
            "density_level": spec["density_level"],
            "actual_density": (
                spec["shelves_per_block"]
                / float(spec["block_size"] ** 2)
            ),
            "shelves_per_block": spec["shelves_per_block"],
            "corridor_policy": "hard_no_drop",
            "block_origins": [
                list(block[0]) for block in variant["blocks"]
            ],
        },
        "factorial_design": {
            "pairing_key": spec["pairing_key"],
            "pairing_seed": spec["pairing_seed"],
            "layout_policy": spec["layout_policy"],
            "robot_start_policy": spec["robot_start_policy"],
            "density_level": spec["density_level"],
            "agent_level": spec["agent_level"],
            "task_profile": spec["task_profile"],
            "goal_mode": spec["goal_mode"],
        },
        "task_profile": {
            "relation_basis": "certificate_assignment",
            "relation_counts": spec["relation_counts"],
            "core_goals": metadata["core_goal_count"],
            "goal_mode": spec["goal_mode"],
            "goal_pool_size": spec["goal_pool_size"],
            "certificate_guarantee": "validator_checked",
            "certificate_steps": metadata["certificate_steps"],
            "certificate_sha256": metadata["certificate_sha256"],
        },
        "robots": [list(cell) for cell in ins.robots],
        "shelves": [list(cell) for cell in ins.shelves],
        "targets": [],
        "flags": {
            "remove_on_complete": False,
            "robots_return_to_rest": False,
        },
    }
    if ins.goal_pool is not None:
        data["goal_pool"] = [list(cell) for cell in ins.goal_pool]
    for target in ins.targets:
        entry = {"id": target.id, "start": list(target.start)}
        if ins.goal_pool is not None:
            entry["goals"] = "pool"
        else:
            entry["goal"] = list(target.goal)
        data["targets"].append(entry)
    return yaml.safe_dump(data, sort_keys=False)


def generate_suite(output_dir: Path, seed: int = 0) -> Dict:
    output_dir = Path(output_dir)
    instances_dir = output_dir / "instances"
    certificates_dir = output_dir / "certificates"
    instances_dir.mkdir(parents=True, exist_ok=True)
    certificates_dir.mkdir(parents=True, exist_ok=True)

    records = []
    for family_id in MAP_FAMILIES:
        for profile in TASK_PROFILES:
            for goal_mode in GOAL_MODES:
                for variant in materialize_pairing_group(
                    family_id, profile, goal_mode, seed
                ):
                    spec = variant["spec"]
                    yaml_path = instances_dir / "{}.yaml".format(spec["id"])
                    certificate_path = certificates_dir / "{}.plan".format(
                        spec["id"]
                    )
                    yaml_path.write_text(
                        _variant_yaml(variant), encoding="utf-8"
                    )
                    certificate_path.write_text(
                        _plan_text(variant["certificate"]),
                        encoding="utf-8",
                    )
                    records.append(
                        {
                            **spec,
                            **variant["metadata"],
                            "yaml": "instances/{}.yaml".format(spec["id"]),
                        }
                    )
    records.sort(key=lambda row: row["id"])
    if len(records) != 432:
        raise AssertionError("generated suite must contain 432 cases")
    (output_dir / "manifest.json").write_text(
        json.dumps(records, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return {
        "case_count": len(records),
        "manifest": output_dir / "manifest.json",
        "records": records,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            HERE
            / "viz_web"
            / "warehouse_case_proposal"
            / "factorial_suite"
        ),
    )
    parser.add_argument("--seed", type=int, default=0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary = generate_suite(args.output_dir, seed=args.seed)
    print("cases={}".format(summary["case_count"]))
    print("manifest={}".format(summary["manifest"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
