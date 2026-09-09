#!/usr/bin/env python3
"""Generate high-density channel maps with depth-stratified target shelves.

The published 509-case benchmark is intentionally left untouched.  This
versioned stress suite fills every storage block uniformly, scrambles shelf
identities with validator-approved sliding moves, and selects target shelves
from explicit edge/inner/deep bands.  Reversing the scramble is a feasibility
certificate, but the certificate is never part of a benchmark instance.
"""

import argparse
import hashlib
import html
import json
import random
import sys
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ddbench.instance import Cell, Instance, Target, save_instance
from ddbench.validator import (
    State,
    apply_joint_action,
    initial_state,
    validate_plan,
)
from generate_warehouse_block_sample import (
    block_geometry,
    choose_robots,
    write_html,
)


HEIGHT = 20
WIDTH = 20
AISLE_WIDTH = 1
ROBOTS = 8
TARGETS = 12
SEED = 0

DEPTH_STRATIFIED = "depth_stratified"
INTERIOR_TO_EDGE = "interior_to_edge"
INTERIOR_TO_BLOCK_EDGE_SET = "interior_to_block_edge_set"
TARGET_PROFILES = (
    DEPTH_STRATIFIED,
    INTERIOR_TO_EDGE,
    INTERIOR_TO_BLOCK_EDGE_SET,
)

# Counts, rather than rounded percentages, make every generated density
# explicit.  The 3x3 family cannot exceed 8/9 without removing every vacancy.
DENSE_SHELF_COUNTS = {
    3: (7, 8),
    4: (12, 14, 15),
    9: (61, 71, 76),
}


def cell_depth(cell: Cell, block_size: int) -> int:
    period = block_size + AISLE_WIDTH
    local_r = cell[0] % period - AISLE_WIDTH
    local_c = cell[1] % period - AISLE_WIDTH
    return min(
        local_r,
        local_c,
        block_size - 1 - local_r,
        block_size - 1 - local_c,
    )


def _neighbors(
    cell: Cell,
    height: int = HEIGHT,
    width: int = WIDTH,
) -> List[Cell]:
    row, col = cell
    output = []
    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        nxt = (row + dr, col + dc)
        if 0 <= nxt[0] < height and 0 <= nxt[1] < width:
            output.append(nxt)
    return output


def _bfs_path(
    start: Cell,
    goal: Cell,
    blocked: Set[Cell],
    rng: random.Random,
    height: int = HEIGHT,
    width: int = WIDTH,
) -> List[Cell]:
    if start == goal:
        return []
    parent = {start: None}
    queue = deque([start])
    while queue:
        current = queue.popleft()
        neighbors = _neighbors(current, height=height, width=width)
        rng.shuffle(neighbors)
        for nxt in neighbors:
            if nxt in blocked or nxt in parent:
                continue
            parent[nxt] = current
            if nxt == goal:
                queue.clear()
                break
            queue.append(nxt)
    if goal not in parent:
        raise ValueError(
            "no lower-deck route from {} to {}".format(start, goal)
        )
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = parent[current]
    path.reverse()
    return path


def _joint(action: Tuple, robots: int) -> List[Tuple]:
    return [action] + [("wait",)] * (robots - 1)


def _apply(
    ins: Instance,
    state: State,
    action: Tuple,
    trajectory: List[Tuple[Tuple[Cell, ...], List[Tuple]]],
) -> State:
    joint = _joint(action, len(ins.robots))
    trajectory.append((tuple(state.robots), joint))
    return apply_joint_action(ins, state, joint)


def _route_active(
    ins: Instance,
    state: State,
    goal: Cell,
    parked: Set[Cell],
    rng: random.Random,
    trajectory: List[Tuple[Tuple[Cell, ...], List[Tuple]]],
    height: int = HEIGHT,
    width: int = WIDTH,
) -> State:
    for cell in _bfs_path(
        state.robots[0],
        goal,
        parked,
        rng,
        height=height,
        width=width,
    ):
        state = _apply(ins, state, ("move", cell), trajectory)
    return state


def _invert_trajectory(
    trajectory: Sequence[Tuple[Tuple[Cell, ...], Sequence[Tuple]]]
) -> List[List[Tuple]]:
    witness = []
    for robots_before, joint in reversed(trajectory):
        inverse = []
        for robot, action in enumerate(joint):
            if action[0] == "wait":
                inverse.append(("wait",))
            elif action[0] == "move":
                inverse.append(("move", robots_before[robot]))
            elif action[0] == "lift":
                inverse.append(("drop",))
            elif action[0] == "drop":
                inverse.append(("lift",))
            else:
                raise ValueError("unknown action {}".format(action))
        witness.append(inverse)
    return witness


def _select_targets(
    all_targets: Sequence[Target],
    positions: Dict[str, Cell],
    block_size: int,
    rng: random.Random,
    target_profile: str = DEPTH_STRATIFIED,
    n_targets: int = TARGETS,
    blocks: Optional[Sequence[Sequence[Cell]]] = None,
) -> Optional[List[Target]]:
    block_by_cell = {}
    if blocks is not None:
        for block_index, block in enumerate(blocks):
            for cell in block:
                block_by_cell[tuple(cell)] = block_index
    records = []
    for target in all_targets:
        start = tuple(positions[target.id])
        goal = tuple(target.goal)
        if start == goal:
            continue
        records.append(
            {
                "id": target.id,
                "start": start,
                "goal": goal,
                "start_depth": cell_depth(start, block_size),
                "goal_depth": cell_depth(goal, block_size),
                "start_block": block_by_cell.get(start),
                "goal_block": block_by_cell.get(goal),
            }
        )

    if target_profile == INTERIOR_TO_BLOCK_EDGE_SET:
        if blocks is None:
            raise ValueError(
                "interior_to_block_edge_set requires block geometry"
            )
        by_block = {}
        for row in records:
            if (
                row["start_depth"] >= 1
                and row["goal_depth"] == 0
                and row["start_block"] == row["goal_block"]
            ):
                by_block.setdefault(row["start_block"], []).append(row)
        block_order = sorted(by_block)
        rng.shuffle(block_order)
        for rows in by_block.values():
            rng.shuffle(rows)
            rows.sort(key=lambda row: -row["start_depth"])
        selected = []
        while len(selected) < n_targets:
            progressed = False
            for block_index in block_order:
                candidates = by_block[block_index]
                if not candidates:
                    continue
                selected.append(candidates.pop(0))
                progressed = True
                if len(selected) == n_targets:
                    break
            if not progressed:
                return None
    elif target_profile == INTERIOR_TO_EDGE:
        candidates = [
            row
            for row in records
            if row["start_depth"] >= 1 and row["goal_depth"] == 0
        ]
        rng.shuffle(candidates)
        candidates.sort(
            key=lambda row: -row["start_depth"]
        )
        if len(candidates) < n_targets:
            return None
        selected = candidates[:n_targets]
    elif target_profile == DEPTH_STRATIFIED:
        if block_size == 9:
            deep = n_targets // 3
            inner = n_targets // 3
            bands = (
                ("deep", deep, lambda row: row["start_depth"] >= 2),
                ("inner", inner, lambda row: row["start_depth"] == 1),
                (
                    "edge",
                    n_targets - deep - inner,
                    lambda row: row["start_depth"] == 0,
                ),
            )
        else:
            bands = (
                ("inner", n_targets // 2,
                 lambda row: row["start_depth"] >= 1),
                ("edge", n_targets - n_targets // 2,
                 lambda row: row["start_depth"] == 0),
            )

        selected = []
        selected_ids = set()
        for _name, quota, predicate in bands:
            candidates = [
                row
                for row in records
                if row["id"] not in selected_ids and predicate(row)
            ]
            rng.shuffle(candidates)
            candidates.sort(
                key=lambda row: (
                    -row["goal_depth"],
                    -row["start_depth"],
                )
            )
            if len(candidates) < quota:
                return None
            chosen = candidates[:quota]
            selected.extend(chosen)
            selected_ids.update(row["id"] for row in chosen)

        if sum(
            row["goal_depth"] >= 1 for row in selected
        ) < n_targets // 3:
            return None
    else:
        raise ValueError(
            "unsupported target profile {}".format(target_profile)
        )

    rng.shuffle(selected)
    targets = []
    for index, row in enumerate(selected):
        if target_profile == INTERIOR_TO_BLOCK_EDGE_SET:
            goals = sorted(
                tuple(cell)
                for cell in blocks[row["start_block"]]
                if cell_depth(tuple(cell), block_size) == 0
            )
            targets.append(
                Target(
                    id="b{}".format(index),
                    start=tuple(row["start"]),
                    goal=goals[0],
                    goals=goals,
                )
            )
        else:
            targets.append(
                Target(
                    id="b{}".format(index),
                    start=tuple(row["start"]),
                    goal=tuple(row["goal"]),
                )
            )
    return targets


def _plan_text(plan: Sequence[Sequence[Tuple]]) -> str:
    def token(action: Tuple) -> str:
        if action[0] == "wait":
            return "w"
        if action[0] == "move":
            return "m {} {}".format(*action[1])
        if action[0] == "lift":
            return "l"
        if action[0] == "drop":
            return "d"
        raise ValueError("unknown action {}".format(action))

    return "\n".join(
        ";".join(token(action) for action in joint)
        for joint in plan
    ) + "\n"


def build_dense_case(
    block_size: int,
    shelves_per_block: int,
    seed: int = SEED,
    target_profile: str = DEPTH_STRATIFIED,
    height: int = HEIGHT,
    width: int = WIDTH,
    n_robots: int = ROBOTS,
    n_targets: int = TARGETS,
) -> Dict:
    if block_size not in DENSE_SHELF_COUNTS:
        raise ValueError("unsupported block size {}".format(block_size))
    if shelves_per_block not in DENSE_SHELF_COUNTS[block_size]:
        raise ValueError(
            "unsupported shelf count {} for block {}".format(
                shelves_per_block, block_size
            )
        )
    if target_profile not in TARGET_PROFILES:
        raise ValueError(
            "unsupported target profile {}".format(target_profile)
        )
    if n_robots < 1 or n_targets < 1:
        raise ValueError("robots and targets must be positive")

    blocks = block_geometry(
        height, width, block_size, AISLE_WIDTH
    )
    storage = {cell for block in blocks for cell in block}
    corridor = {
        (row, col)
        for row in range(height)
        for col in range(width)
        if (row, col) not in storage
    }
    density = shelves_per_block / float(block_size * block_size)
    if target_profile == INTERIOR_TO_BLOCK_EDGE_SET:
        name_prefix = "dense_channel_bedge"
    elif target_profile == INTERIOR_TO_EDGE:
        name_prefix = "dense_channel_i2e"
    else:
        name_prefix = "dense_channel"
    name = (
        "{}_h{}w{}_b{}_a{}_s{}of{}_r{}_t{}_seed{}".format(
            name_prefix,
            height,
            width,
            block_size,
            AISLE_WIDTH,
            shelves_per_block,
            block_size * block_size,
            n_robots,
            n_targets,
            seed,
        )
    )

    for attempt in range(12):
        attempt_seed = (
            seed * 1000003
            + block_size * 10007
            + shelves_per_block * 1009
            + attempt * 104729
        )
        rng = random.Random(attempt_seed)
        shelf_cells = []
        for block in blocks:
            shelf_cells.extend(
                rng.sample(list(block), shelves_per_block)
            )
        shelf_cells = sorted(shelf_cells)
        all_targets = [
            Target(id="s{}".format(index), start=cell, goal=cell)
            for index, cell in enumerate(shelf_cells)
        ]
        robots = choose_robots(
            height, width, storage, n_robots, rng
        )
        goal_instance = Instance(
            grid=[[False] * width for _ in range(height)],
            robots=robots,
            shelves=shelf_cells,
            targets=all_targets,
            name=name,
            storage_cells=set(storage),
        )
        errors = goal_instance.validate_static()
        if errors:
            raise ValueError("{} goal state: {}".format(name, errors))

        state = initial_state(goal_instance)
        trajectory = []
        parked = set(robots[1:])
        slides_per_block = max(
            36, block_size * block_size * (4 + attempt)
        )
        for block in blocks:
            block_set = set(block)
            previous = None
            for _ in range(slides_per_block):
                occupied = state.shelf_cells()
                vacancies = block_set - occupied
                options = []
                for vacancy in vacancies:
                    for shelf in _neighbors(
                        vacancy, height=height, width=width
                    ):
                        if shelf in block_set and shelf in occupied:
                            option = (shelf, vacancy)
                            if previous is not None and option == previous:
                                continue
                            options.append(option)
                if not options:
                    break
                shelf, vacancy = rng.choice(options)
                state = _route_active(
                    goal_instance,
                    state,
                    shelf,
                    parked,
                    rng,
                    trajectory,
                    height=height,
                    width=width,
                )
                state = _apply(
                    goal_instance, state, ("lift",), trajectory
                )
                state = _apply(
                    goal_instance,
                    state,
                    ("move", vacancy),
                    trajectory,
                )
                state = _apply(
                    goal_instance, state, ("drop",), trajectory
                )
                previous = (vacancy, shelf)

        parking_candidates = sorted(corridor - parked)
        parking_candidates.sort(
            key=lambda cell: (
                abs(cell[0] - state.robots[0][0])
                + abs(cell[1] - state.robots[0][1]),
                cell,
            )
        )
        state = _route_active(
            goal_instance,
            state,
            parking_candidates[0],
            parked,
            rng,
            trajectory,
            height=height,
            width=width,
        )
        positions = dict(state.target_pos)
        targets = _select_targets(
            all_targets,
            positions,
            block_size,
            rng,
            target_profile=target_profile,
            n_targets=n_targets,
            blocks=blocks,
        )
        if targets is None:
            continue

        instance = Instance(
            grid=[[False] * width for _ in range(height)],
            robots=[tuple(robot) for robot in state.robots],
            shelves=sorted(positions.values()),
            targets=targets,
            name=name,
            storage_cells=set(storage),
        )
        errors = instance.validate_static()
        if errors:
            raise ValueError("{} scrambled state: {}".format(name, errors))
        witness = _invert_trajectory(trajectory)
        ok, errors, _ = validate_plan(instance, witness)
        if not ok:
            raise ValueError(
                "{} invalid reverse witness: {}".format(name, errors)
            )

        start_depths = [
            cell_depth(target.start, block_size)
            for target in targets
        ]
        goal_depths = [
            cell_depth(target.goal, block_size)
            for target in targets
        ]
        case = {
            "name": name,
            "height": height,
            "width": width,
            "block_size": block_size,
            "aisle_width": AISLE_WIDTH,
            "requested_density": density,
            "density": density,
            "shelves_per_block": shelves_per_block,
            "blocks": blocks,
            "storage": storage,
            "corridor": corridor,
            "robots": list(instance.robots),
            "shelves": list(instance.shelves),
            "targets": [
                {
                    "id": target.id,
                    "start": tuple(target.start),
                    "goal": tuple(target.goal),
                    **(
                        {
                            "eligible_goals": [
                                tuple(goal)
                                for goal in target.eligible_goals()
                            ]
                        }
                        if len(target.eligible_goals()) > 1
                        else {}
                    ),
                }
                for target in targets
            ],
        }
        return {
            "instance": instance,
            "witness": witness,
            "case": case,
            "blocks": blocks,
            "storage": storage,
            "corridor": corridor,
            "attempt": attempt,
            "actual_density": density,
            "target_profile": target_profile,
            "n_robots": n_robots,
            "n_targets": n_targets,
            "start_depths": start_depths,
            "goal_depths": goal_depths,
        }
    raise RuntimeError(
        "could not generate {} case {} after retries".format(
            target_profile, name
        )
    )


def _manifest_row(generated: Dict) -> Dict:
    ins = generated["instance"]
    block_size = generated["case"]["block_size"]
    shelves_per_block = generated["case"]["shelves_per_block"]
    target_profile = generated["target_profile"]
    witness_text = _plan_text(generated["witness"])
    return {
        "id": ins.name,
        "name": ins.name,
        "map_family": (
            "dense-channel-bedge-b{}".format(block_size)
            if target_profile == INTERIOR_TO_BLOCK_EDGE_SET
            else "dense-channel-i2e-b{}".format(block_size)
            if target_profile == INTERIOR_TO_EDGE
            else "dense-channel-b{}".format(block_size)
        ),
        "height": generated["case"]["height"],
        "width": generated["case"]["width"],
        "block_size": block_size,
        "density_level": "{}/{}".format(
            shelves_per_block, block_size * block_size
        ),
        "actual_density": generated["actual_density"],
        "agent_level": generated.get("agent_level", "baseline"),
        "robots": generated["n_robots"],
        "targets": generated["n_targets"],
        "task_profile": target_profile,
        "goal_mode": (
            "per_block_edge_set"
            if target_profile == INTERIOR_TO_BLOCK_EDGE_SET
            else "singleton"
        ),
        "goal_pool_size": 0,
        **(
            {
                "goals_per_target": min(
                    len(target.eligible_goals())
                    for target in ins.targets
                ),
                "eligible_goal_union_size": len(
                    {
                        goal
                        for target in ins.targets
                        for goal in target.eligible_goals()
                    }
                ),
            }
            if target_profile == INTERIOR_TO_BLOCK_EDGE_SET
            else {}
        ),
        "shelves_per_block": shelves_per_block,
        "start_edge": sum(
            depth == 0 for depth in generated["start_depths"]
        ),
        "start_inner": sum(
            depth >= 1 for depth in generated["start_depths"]
        ),
        "start_deep": sum(
            depth >= 2 for depth in generated["start_depths"]
        ),
        "goal_edge": sum(
            depth == 0 for depth in generated["goal_depths"]
        ),
        "goal_inner": sum(
            depth >= 1 for depth in generated["goal_depths"]
        ),
        "generation_attempt": generated["attempt"],
        "certificate_steps": len(generated["witness"]),
        "certificate_sha256": hashlib.sha256(
            witness_text.encode("utf-8")
        ).hexdigest(),
        "yaml": "instances/{}.yaml".format(ins.name),
        "certificate": "certificates/{}.plan".format(ins.name),
        "preview": "cases/{}.html".format(ins.name),
    }


def _write_index(
    rows: Sequence[Dict],
    output_dir: Path,
    target_profile: str = DEPTH_STRATIFIED,
) -> None:
    if target_profile == DEPTH_STRATIFIED:
        cards = []
        for row in rows:
            cards.append(
                """<article><h2>{name}</h2>
<p>密度 <b>{density:.1%}</b>（{count}/{cells}） · target 起点：
edge {start_edge} / inner {start_inner} / deep {start_deep} · goal inner
{goal_inner} · witness {steps} 拍</p>
<p><a href="{preview}">查看初始布局</a> ·
<a href="{yaml}">YAML</a> ·
<a href="{certificate}">可解性 witness</a></p></article>""".format(
                    name=html.escape(row["name"]),
                    density=row["actual_density"],
                    count=row["shelves_per_block"],
                    cells=row["block_size"] ** 2,
                    start_edge=row["start_edge"],
                    start_inner=row["start_inner"],
                    start_deep=row["start_deep"],
                    goal_inner=row["goal_inner"],
                    steps=row["certificate_steps"],
                    preview=html.escape(row["preview"]),
                    yaml=html.escape(row["yaml"]),
                    certificate=html.escape(row["certificate"]),
                )
            )
        page = """<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Dense-channel benchmark V2</title><style>
body{{margin:auto;max-width:1180px;padding:30px;background:#09111d;color:#edf5ff;
font:15px/1.65 system-ui}}a{{color:#77b7ff}}.lead{{color:#a8bacd}}
.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(310px,1fr));
gap:14px}}article{{background:#101f32;border:1px solid #29425d;border-radius:12px;
padding:16px}}h2{{font-size:15px;overflow-wrap:anywhere}}b{{color:#58d6b3}}
</style></head><body><h1>Dense-channel benchmark V2</h1>
<p class="lead">不改写已发布 509 例。本页新增高密度通道压力集，目标货架按
storage-block 深度分层选择；每例都附有 authoritative validator 验证的逆向
witness，但 benchmark solver 不读取 witness。</p><div class="grid">{cards}
</div></body></html>""".format(cards="\n".join(cards))
        (output_dir / "index.html").write_text(page, encoding="utf-8")
        return

    cards = []
    for row in rows:
        depth_text = (
            "target 起点：inner {start_inner} / deep {start_deep} · "
            "goal edge {goal_edge}"
        ).format(**row)
        cards.append(
            """<article><h2>{name}</h2>
<p>密度 <b>{density:.1%}</b>（{count}/{cells}） · {depth_text} ·
witness {steps} 拍</p>
<p><a href="{preview}">查看初始布局</a> ·
<a href="{yaml}">YAML</a> ·
<a href="{certificate}">可解性 witness</a></p></article>""".format(
                name=html.escape(row["name"]),
                density=row["actual_density"],
                count=row["shelves_per_block"],
                cells=row["block_size"] ** 2,
                depth_text=depth_text,
                steps=row["certificate_steps"],
                preview=html.escape(row["preview"]),
                yaml=html.escape(row["yaml"]),
                certificate=html.escape(row["certificate"]),
            )
        )
    title = "Dense-channel interior-to-edge benchmark V1"
    lead = (
        "高密度货架块使用单格通道。每个目标货箱都从 storage block "
        "内部出发，目的地固定在 block 边缘；每例都附有 authoritative "
        "validator 验证的逆向 witness，但 benchmark solver 不读取 "
        "witness。"
    )
    page = """<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{title}</title><style>
body{{margin:auto;max-width:1180px;padding:30px;background:#09111d;color:#edf5ff;
font:15px/1.65 system-ui}}a{{color:#77b7ff}}.lead{{color:#a8bacd}}
.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(310px,1fr));
gap:14px}}article{{background:#101f32;border:1px solid #29425d;border-radius:12px;
padding:16px}}h2{{font-size:15px;overflow-wrap:anywhere}}b{{color:#58d6b3}}
</style></head><body><h1>{title}</h1>
<p class="lead">{lead}</p><div class="grid">{cards}
</div></body></html>""".format(
        title=html.escape(title),
        lead=html.escape(lead),
        cards="\n".join(cards),
    )
    (output_dir / "index.html").write_text(page, encoding="utf-8")


def generate_suite(
    output_dir: Path,
    target_profile: str = DEPTH_STRATIFIED,
) -> List[Dict]:
    if target_profile not in TARGET_PROFILES:
        raise ValueError(
            "unsupported target profile {}".format(target_profile)
        )
    output_dir = Path(output_dir)
    instances_dir = output_dir / "instances"
    certificates_dir = output_dir / "certificates"
    cases_dir = output_dir / "cases"
    for directory in (instances_dir, certificates_dir, cases_dir):
        directory.mkdir(parents=True, exist_ok=True)

    rows = []
    for block_size in sorted(DENSE_SHELF_COUNTS):
        for shelves_per_block in DENSE_SHELF_COUNTS[block_size]:
            generated = build_dense_case(
                block_size=block_size,
                shelves_per_block=shelves_per_block,
                seed=SEED,
                target_profile=target_profile,
            )
            ins = generated["instance"]
            save_instance(
                ins, instances_dir / "{}.yaml".format(ins.name)
            )
            certificate_path = (
                certificates_dir / "{}.plan".format(ins.name)
            )
            certificate_path.write_text(
                _plan_text(generated["witness"]), encoding="utf-8"
            )
            write_html(
                generated["case"],
                cases_dir / "{}.html".format(ins.name),
            )
            rows.append(_manifest_row(generated))

    (output_dir / "manifest.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    _write_index(rows, output_dir, target_profile=target_profile)
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            Path(__file__).resolve().parent
            / "viz_web"
            / "dense_channel_suite_v2_20260906"
        ),
    )
    parser.add_argument(
        "--target-profile",
        choices=TARGET_PROFILES,
        default=DEPTH_STRATIFIED,
    )
    args = parser.parse_args()
    rows = generate_suite(
        args.output_dir,
        target_profile=args.target_profile,
    )
    print(
        "generated {} {} dense-channel cases under {}".format(
            len(rows), args.target_profile, args.output_dir
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
