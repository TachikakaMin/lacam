"""Paper-protocol instance generators (CREST paper arXiv:2603.28803 §V-A).

Layouts:
  R2R  Random-to-Random (from DD-MAPD): 2x2 shelf blocks sampled to density,
       0.1*n^2 relocation targets, random free-cell goals, perimeter agents.
       (= ddbench.generators.ddmapd_instance)
  S2W  Staging-to-Warehouse: shelves start packed in a staging zone and are
       relocated to designated storage slots (all shelves rearranged).
  DnE  Distributed-and-Exchange: designated storage slots; half the shelves
       start inside storage slots (keep them), the rest start at random
       non-storage cells and must move into remaining slots.

The paper's exact generator is not published; these follow the textual
descriptions and Table I statistics (shelf counts / rearranged counts).
"""

import random
from typing import List, Optional, Set, Tuple

from .instance import Cell, Instance, Target


def _perimeter(h: int, w: int) -> List[Cell]:
    return (
        [(0, c) for c in range(w)]
        + [(h - 1, c) for c in range(w)]
        + [(r, 0) for r in range(1, h - 1)]
        + [(r, w - 1) for r in range(1, h - 1)]
    )


def _storage_slots(h: int, w: int, count: int) -> List[Cell]:
    """Designated storage: warehouse-style 2x2 blocks with 1-wide aisles,
    laid out row-major starting at (2,2), until `count` slots collected."""
    slots: List[Cell] = []
    r = 2
    while r + 1 < h - 2 and len(slots) < count:
        c = 2
        while c + 1 < w - 2 and len(slots) < count:
            for cell in ((r, c), (r, c + 1), (r + 1, c), (r + 1, c + 1)):
                if len(slots) < count:
                    slots.append(cell)
            c += 3
        r += 3
    if len(slots) < count:
        raise ValueError(f"map too small for {count} storage slots")
    return slots


def s2w_instance(
    height: int,
    width: int,
    n_robots: int,
    n_shelves: int,
    seed: int = 0,
    name: str = "",
) -> Instance:
    """Staging-to-Warehouse: all shelves rearranged (targets)."""
    rng = random.Random(seed)
    grid = [[False] * width for _ in range(height)]
    slots = _storage_slots(height, width, n_shelves)
    slot_set = set(slots)

    # staging zone: densely pack shelves column-by-column from the left edge,
    # skipping storage slots and the perimeter (agents start there).
    staging: List[Cell] = []
    for c in range(1, width - 1):
        for r in range(1, height - 1):
            if (r, c) in slot_set:
                continue
            staging.append((r, c))
            if len(staging) == n_shelves:
                break
        if len(staging) == n_shelves:
            break
    if len(staging) < n_shelves:
        raise ValueError("map too small for staging zone")

    goals = list(slots)
    rng.shuffle(goals)  # each shelf may end at any designated location
    used = set(staging) | slot_set
    robots = rng.sample([p for p in _perimeter(height, width) if p not in used],
                        n_robots)
    ins = Instance(
        grid=grid,
        robots=robots,
        shelves=list(staging),
        targets=[
            Target(id=f"b{i}", start=staging[i], goal=goals[i])
            for i in range(n_shelves)
        ],
        name=name or f"s2w_h{height}w{width}r{n_robots}s{n_shelves}_seed{seed}",
    )
    errs = ins.validate_static()
    if errs:
        raise ValueError(f"s2w instance invalid: {errs}")
    return ins


def dne_instance(
    height: int,
    width: int,
    n_robots: int,
    n_shelves: int,
    seed: int = 0,
    name: str = "",
    n_rearranged: Optional[int] = None,
) -> Instance:
    """Distributed-and-Exchange: `n_rearranged` shelves start at random
    non-storage cells and must move into free storage slots; the rest start
    inside storage slots and stay (anonymous)."""
    rng = random.Random(seed)
    grid = [[False] * width for _ in range(height)]
    slots = _storage_slots(height, width, n_shelves)
    slot_set = set(slots)

    if n_rearranged is None:
        n_rearranged = n_shelves - n_shelves // 2
    n_inside = n_shelves - n_rearranged
    inside = rng.sample(slots, n_inside)
    inside_set = set(inside)
    non_storage = [
        (r, c)
        for r in range(1, height - 1)
        for c in range(1, width - 1)
        if (r, c) not in slot_set
    ]
    outside = rng.sample(non_storage, n_rearranged)

    remaining_slots = [s for s in slots if s not in inside_set]
    rng.shuffle(remaining_slots)

    shelves = inside + outside
    targets = [
        Target(id=f"b{i}", start=outside[i], goal=remaining_slots[i])
        for i in range(len(outside))
    ]
    # shelves already inside storage stay anonymous (goal = stay put)
    used = set(shelves) | slot_set
    robots = rng.sample([p for p in _perimeter(height, width) if p not in used],
                        n_robots)
    ins = Instance(
        grid=grid,
        robots=robots,
        shelves=shelves,
        targets=targets,
        name=name or f"dne_h{height}w{width}r{n_robots}s{n_shelves}_seed{seed}",
    )
    errs = ins.validate_static()
    if errs:
        raise ValueError(f"dne instance invalid: {errs}")
    return ins
