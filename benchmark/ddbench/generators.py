"""Benchmark instance generators (design.md 8.2).

Two protocols:

1. scramble_instance: start from the goal configuration, apply k random legal
   *inverse-closed* primitive joint actions via the validator.  Every primitive
   has an inverse (Move(v)<->Move(u), Lift<->Drop, Wait<->Wait) and the rule
   table is time-reversal symmetric, so the reversed trajectory is itself a
   legal solution: feasibility by construction.  k is the difficulty knob.

2. ddmapd_instance: DD-MAPD paper protocol — sample 2x2 shelf blocks to a given
   density, choose 0.1*n^2-style relocation targets, perimeter agent starts.
"""

import random
from typing import Dict, List, Optional, Set, Tuple

from .instance import Cell, Instance, Target
from .validator import (
    ANON,
    State,
    apply_joint_action,
    initial_state,
    legal_actions_for_robot,
    TransitionError,
)


def _empty_grid(h: int, w: int) -> List[List[bool]]:
    return [[False] * w for _ in range(h)]


def scramble_instance(
    height: int,
    width: int,
    n_robots: int,
    n_shelves: int,
    n_targets: int,
    k: int,
    seed: int = 0,
    walls: Optional[List[List[bool]]] = None,
    name: str = "",
) -> Instance:
    """Generate a feasible instance by scrambling backwards from the goal.

    Construction:
      - goal configuration: shelves placed at random distinct free cells; the
        first n_targets of them are the labeled targets AT their goals;
        robots at random distinct free cells, all free (kappa=bot).
      - apply k random legal joint actions (using the validator as the only
        legality oracle) to obtain the start configuration.
      - the instance start = scrambled state; target goals = original cells.

    Feasibility by construction: the reverse action sequence returns every
    shelf (in particular every target) to its goal cell, grounded.
    """
    rng = random.Random(seed)
    grid = walls if walls is not None else _empty_grid(height, width)
    free = [
        (r, c)
        for r in range(len(grid))
        for c in range(len(grid[0]))
        if not grid[r][c]
    ]
    if n_shelves + 0 > len(free):
        raise ValueError("too many shelves for free cells")
    shelf_cells = rng.sample(free, n_shelves)
    robot_cells = rng.sample(free, n_robots)  # lower deck: may coincide w/ shelf

    targets = [
        Target(id=f"b{i}", start=shelf_cells[i], goal=shelf_cells[i])
        for i in range(n_targets)
    ]
    goal_ins = Instance(
        grid=grid,
        robots=robot_cells,
        shelves=list(shelf_cells),
        targets=targets,
        name=name or f"scramble_h{height}w{width}r{n_robots}s{n_shelves}t{n_targets}k{k}_seed{seed}",
    )
    errs = goal_ins.validate_static()
    if errs:
        raise ValueError(f"goal configuration invalid: {errs}")

    s = initial_state(goal_ins)
    for _ in range(k):
        s = _random_legal_step(goal_ins, s, rng)
    # settle: force-drop every carried shelf so the start state has all shelves
    # grounded (start states are grounded configurations by definition).
    s = _settle(goal_ins, s, rng)

    start_targets = dict(s.target_pos)
    ins = Instance(
        grid=grid,
        robots=[tuple(q) for q in s.robots],
        shelves=sorted(set(s.anon_occ) | set(start_targets.values())),
        targets=[
            Target(id=t.id, start=start_targets[t.id], goal=tuple(t.goal))
            for t in targets
        ],
        name=goal_ins.name,
    )
    errs = ins.validate_static()
    if errs:
        raise ValueError(f"scrambled instance invalid: {errs}")
    return ins


def _random_legal_step(ins: Instance, s: State, rng: random.Random) -> State:
    """Sample one random legal joint action (rejection sampling per robot with
    validator as final arbiter)."""
    for _attempt in range(200):
        joint = []
        for i in range(len(s.robots)):
            acts = legal_actions_for_robot(ins, s, i)
            # bias against wait to actually scramble
            weights = [1 if a[0] == "wait" else 4 for a in acts]
            joint.append(rng.choices(acts, weights=weights, k=1)[0])
        try:
            return apply_joint_action(ins, s, joint)
        except TransitionError:
            continue
    # fall back: everyone waits (always legal)
    return apply_joint_action(ins, s, [("wait",)] * len(s.robots))


def _settle(ins: Instance, s: State, rng: random.Random) -> State:
    """Drop every carried shelf (moving off occupied upper cells if needed)."""
    guard = 0
    while any(k is not None for k in s.kappa) and guard < 500:
        guard += 1
        joint = []
        occupied_upper = s.shelf_cells()
        for i in range(len(s.robots)):
            if s.kappa[i] is None:
                joint.append(("wait",))
                continue
            q = s.robots[i]
            # try drop: legal iff no other shelf grounded here
            other_upper = occupied_upper - ({dict(s.target_pos).get(s.kappa[i])} if s.kappa[i] != ANON else set())
            if q not in other_upper or (s.kappa[i] != ANON and dict(s.target_pos).get(s.kappa[i]) == q):
                joint.append(("drop",))
            else:
                moves = [("move", v) for v in ins.neighbors(q)]
                rng.shuffle(moves)
                joint.append(moves[0] if moves else ("wait",))
        try:
            s = apply_joint_action(ins, s, joint)
        except TransitionError:
            s = _random_legal_step(ins, s, rng)
    if any(k is not None for k in s.kappa):
        raise RuntimeError("settle failed: robots still carrying")
    return s


def scramble_with_witness(
    height: int,
    width: int,
    n_robots: int,
    n_shelves: int,
    n_targets: int,
    k: int,
    seed: int = 0,
    walls: Optional[List[List[bool]]] = None,
    name: str = "",
):
    """Like scramble_instance, but also returns the witness solution plan
    (the reversed scramble trajectory), which must satisfy the validator.

    Implementation trick: scramble a FULLY-LABELED twin instance (every shelf
    is a target) so the validator itself tracks shelf identities; anonymize
    afterwards.  Legality is unaffected (labeled vs ANON changes no rule).
    `anon_goals` then records the exact witness pairing start->goal for the
    anonymous shelves, so full-goal-layout baselines get a realizable layout.

    Returns (instance, witness_plan).
    """
    rng = random.Random(seed)
    grid = walls if walls is not None else _empty_grid(height, width)
    free = [
        (r, c)
        for r in range(len(grid))
        for c in range(len(grid[0]))
        if not grid[r][c]
    ]
    if n_shelves > len(free):
        raise ValueError("too many shelves for free cells")
    shelf_cells = rng.sample(free, n_shelves)
    robot_cells = rng.sample(free, n_robots)

    inst_name = (
        name
        or f"scramble_h{height}w{width}r{n_robots}s{n_shelves}t{n_targets}k{k}_seed{seed}"
    )
    # fully-labeled twin: every shelf is a target with goal = its cell
    all_targets = [
        Target(id=f"b{i}", start=shelf_cells[i], goal=shelf_cells[i])
        for i in range(n_shelves)
    ]
    labeled_ins = Instance(
        grid=grid,
        robots=robot_cells,
        shelves=list(shelf_cells),
        targets=all_targets,
        name=inst_name,
    )
    errs = labeled_ins.validate_static()
    if errs:
        raise ValueError(f"goal configuration invalid: {errs}")

    s = initial_state(labeled_ins)
    traj: List[Tuple[State, List]] = []
    for _ in range(k):
        s_next, joint = _random_legal_step_traced(labeled_ins, s, rng)
        traj.append((s, joint))
        s = s_next
    s_next, settle_steps = _settle_traced(labeled_ins, s, rng)
    traj.extend(settle_steps)
    s = s_next

    pos = dict(s.target_pos)  # id -> scrambled (start) cell
    # anonymize: only first n_targets stay labeled
    kept = all_targets[:n_targets]
    anon_pairs = [
        [list(pos[t.id]), list(t.goal)] for t in all_targets[n_targets:]
    ]
    ins = Instance(
        grid=grid,
        robots=[tuple(q) for q in s.robots],
        shelves=sorted(pos.values()),
        targets=[
            Target(id=t.id, start=tuple(pos[t.id]), goal=tuple(t.goal))
            for t in kept
        ],
        name=inst_name,
        anon_goals=anon_pairs,
    )
    errs = ins.validate_static()
    if errs:
        raise ValueError(f"scrambled instance invalid: {errs}")
    if all(tuple(t.start) == tuple(t.goal) for t in ins.targets):
        # trivial instance (every target already home): rescramble
        return scramble_with_witness(
            height, width, n_robots, n_shelves, n_targets, k,
            seed=seed + 7919, walls=walls, name=name,
        )

    # witness = reversed trajectory with inverted primitives
    witness: List[List] = []
    for s_before, joint in reversed(traj):
        inv = []
        for i, act in enumerate(joint):
            if act[0] == "wait":
                inv.append(("wait",))
            elif act[0] == "move":
                inv.append(("move", s_before.robots[i]))
            elif act[0] == "lift":
                inv.append(("drop",))
            elif act[0] == "drop":
                inv.append(("lift",))
        witness.append(inv)
    return ins, witness


def _random_legal_step_traced(ins: Instance, s: State, rng: random.Random):
    for _attempt in range(200):
        joint = []
        for i in range(len(s.robots)):
            acts = legal_actions_for_robot(ins, s, i)
            weights = [1 if a[0] == "wait" else 4 for a in acts]
            joint.append(rng.choices(acts, weights=weights, k=1)[0])
        try:
            return apply_joint_action(ins, s, joint), joint
        except TransitionError:
            continue
    joint = [("wait",)] * len(s.robots)
    return apply_joint_action(ins, s, joint), joint


def _settle_traced(ins: Instance, s: State, rng: random.Random):
    steps = []
    guard = 0
    while any(k is not None for k in s.kappa) and guard < 500:
        guard += 1
        joint = []
        for i in range(len(s.robots)):
            if s.kappa[i] is None:
                joint.append(("wait",))
                continue
            joint.append(("drop",))
        try:
            s_next = apply_joint_action(ins, s, joint)
            steps.append((s, joint))
            s = s_next
            continue
        except TransitionError:
            pass
        s_next, joint = _random_legal_step_traced(ins, s, rng)
        steps.append((s, joint))
        s = s_next
    if any(k is not None for k in s.kappa):
        raise RuntimeError("settle failed: robots still carrying")
    return s, steps


def ddmapd_instance(
    height: int,
    width: int,
    n_robots: int,
    block_density: float,
    n_targets: Optional[int] = None,
    seed: int = 0,
    name: str = "",
) -> Instance:
    """DD-MAPD paper protocol (design.md 8.2 item 6):
      - sample 2x2 shelf blocks until shelf density reached;
      - pick ~0.1*n^2 relocation targets among shelves, goals sampled from
        free cells (unoccupied by any shelf);
      - robot starts on the perimeter.
    """
    rng = random.Random(seed)
    grid = _empty_grid(height, width)
    cells_total = height * width
    shelf_target_count = int(block_density * cells_total)

    occ: Set[Cell] = set()
    attempts = 0
    while len(occ) < shelf_target_count and attempts < 20000:
        attempts += 1
        r = rng.randrange(0, height - 1)
        c = rng.randrange(0, width - 1)
        block = {(r, c), (r + 1, c), (r, c + 1), (r + 1, c + 1)}
        if block & occ:
            continue
        occ |= block
    shelves = sorted(occ)

    perimeter = (
        [(0, c) for c in range(width)]
        + [(height - 1, c) for c in range(width)]
        + [(r, 0) for r in range(1, height - 1)]
        + [(r, width - 1) for r in range(1, height - 1)]
    )
    # DD-MAPD well-formedness/safety: agent initial cells must not coincide
    # with shelf pickup or delivery cells (decomposed baselines require safe
    # shelf plans that avoid agent initial locations).
    perimeter_free = [p for p in perimeter if p not in occ]
    robot_starts = rng.sample(perimeter_free, n_robots)
    robot_set = set(robot_starts)

    if n_targets is None:
        n_targets = max(1, int(0.1 * (height * width) ** 0.5) ** 2)
    n_targets = min(n_targets, len(shelves))
    target_starts = rng.sample(shelves, n_targets)
    free_cells = [
        (r, c)
        for r in range(height)
        for c in range(width)
        if (r, c) not in occ and (r, c) not in robot_set
    ]
    goals = rng.sample(free_cells, n_targets)

    ins = Instance(
        grid=grid,
        robots=robot_starts,
        shelves=shelves,
        targets=[
            Target(id=f"b{i}", start=target_starts[i], goal=goals[i])
            for i in range(n_targets)
        ],
        name=name
        or f"ddmapd_h{height}w{width}r{n_robots}d{int(block_density*100)}t{n_targets}_seed{seed}",
    )
    errs = ins.validate_static()
    if errs:
        raise ValueError(f"ddmapd instance invalid: {errs}")
    return ins
