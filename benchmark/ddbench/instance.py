"""Double-deck warehouse rearrangement benchmark instance format.

YAML instance format (design.md M0):

  map: |            # '.'/free, '@' or 'T' wall  (rows of equal width)
    ....
    .@..
  robots:           # labeled robot start cells [row, col]
    - [0, 0]
  shelves:          # ALL shelf cells (upper deck occupancy), incl. targets
    - [1, 2]
  targets:          # labeled target shelves, subset of shelves by position
    - id: b0
      start: [1, 2]
      goal: [3, 3]
  flags:
    remove_on_complete: false
    robots_return_to_rest: false

Semantics (design.md section 2/3):
  - lower deck: robots; free robot may pass/stay under grounded shelf.
  - upper deck: shelves; grounded shelf static, carried shelf moves with robot.
  - Non-target shelves are anonymous occupancy.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

Cell = Tuple[int, int]  # (row, col)


@dataclass
class Target:
    id: str
    start: Cell
    goal: Cell


@dataclass
class Instance:
    grid: List[List[bool]]  # True = wall
    robots: List[Cell]
    shelves: List[Cell]  # every shelf cell, including target shelf starts
    targets: List[Target]
    flags: Dict[str, bool] = field(default_factory=dict)
    name: str = ""
    # Optional hint for full-goal-layout baselines (CREST/NAT-CBS): exact
    # witness pairing [(anon_start, anon_goal), ...] such that the combined
    # layout {target goals} + anon goals is realizable (scrambler witness).
    # NOT part of our goal condition (anonymous shelves are unconstrained).
    anon_goals: Optional[List[Tuple[Cell, Cell]]] = None

    @property
    def height(self) -> int:
        return len(self.grid)

    @property
    def width(self) -> int:
        return len(self.grid[0]) if self.grid else 0

    def is_wall(self, cell: Cell) -> bool:
        r, c = cell
        return self.grid[r][c]

    def in_bounds(self, cell: Cell) -> bool:
        r, c = cell
        return 0 <= r < self.height and 0 <= c < self.width

    def neighbors(self, cell: Cell) -> List[Cell]:
        r, c = cell
        out = []
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = (r + dr, c + dc)
            if self.in_bounds(nxt) and not self.is_wall(nxt):
                out.append(nxt)
        return out

    def validate_static(self) -> List[str]:
        """Static well-formedness checks. Returns list of error strings."""
        errors = []
        if not self.grid:
            errors.append("empty map")
            return errors
        w = self.width
        for i, row in enumerate(self.grid):
            if len(row) != w:
                errors.append(f"map row {i} width {len(row)} != {w}")
        seen_r = set()
        for i, q in enumerate(self.robots):
            if not self.in_bounds(q) or self.is_wall(q):
                errors.append(f"robot {i} at invalid cell {q}")
            if q in seen_r:
                errors.append(f"robots overlap at {q}")
            seen_r.add(q)
        seen_s = set()
        for i, p in enumerate(self.shelves):
            if not self.in_bounds(p) or self.is_wall(p):
                errors.append(f"shelf {i} at invalid cell {p}")
            if p in seen_s:
                errors.append(f"shelves overlap at {p}")
            seen_s.add(p)
        seen_ids = set()
        seen_goals = set()
        for t in self.targets:
            if t.id in seen_ids:
                errors.append(f"duplicate target id {t.id}")
            seen_ids.add(t.id)
            if tuple(t.start) not in seen_s:
                errors.append(f"target {t.id} start {t.start} is not a shelf cell")
            if not self.in_bounds(t.goal) or self.is_wall(t.goal):
                errors.append(f"target {t.id} goal {t.goal} invalid")
            if tuple(t.goal) in seen_goals:
                errors.append(f"duplicate target goal {t.goal}")
            seen_goals.add(tuple(t.goal))
        return errors


def parse_map_str(map_str: str) -> List[List[bool]]:
    rows = [line for line in map_str.splitlines() if line.strip()]
    return [[ch in "@T#" for ch in row] for row in rows]


def dump_map_str(grid: List[List[bool]]) -> str:
    return "\n".join("".join("@" if x else "." for x in row) for row in grid)


def load_instance(path) -> Instance:
    data = yaml.safe_load(Path(path).read_text())
    # debug.md P0-4: v1 implements default flag semantics only; fail loudly
    # on any non-default value instead of silently ignoring it.
    for key, value in dict(data.get("flags") or {}).items():
        if bool(value):
            raise ValueError(
                f"load_instance: unsupported non-default flag {key!r} "
                "(v1 implements defaults only)"
            )
    grid = parse_map_str(data["map"])
    robots = [tuple(x) for x in data.get("robots", [])]
    shelves = [tuple(x) for x in data.get("shelves", [])]
    targets = [
        Target(id=str(t["id"]), start=tuple(t["start"]), goal=tuple(t["goal"]))
        for t in data.get("targets", [])
    ]
    return Instance(
        grid=grid,
        robots=robots,
        shelves=shelves,
        targets=targets,
        flags=dict(data.get("flags", {})),
        name=str(data.get("name", Path(path).stem)),
        anon_goals=[
            (tuple(pair[0]), tuple(pair[1])) for pair in data["anon_goals"]
        ]
        if data.get("anon_goals")
        else None,
    )


def save_instance(ins: Instance, path) -> None:
    data = {
        "name": ins.name,
        "map": dump_map_str(ins.grid) + "\n",
        "robots": [list(q) for q in ins.robots],
        "shelves": [list(p) for p in ins.shelves],
        "targets": [
            {"id": t.id, "start": list(t.start), "goal": list(t.goal)}
            for t in ins.targets
        ],
        "flags": ins.flags,
    }
    if ins.anon_goals is not None:
        data["anon_goals"] = [
            [list(pair[0]), list(pair[1])] for pair in ins.anon_goals
        ]
    Path(path).write_text(yaml.safe_dump(data, sort_keys=False))
