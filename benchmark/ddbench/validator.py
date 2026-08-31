"""Two-deck transition validator (design.md sections 3.2, 3.3).

State X = (Q_R, Q_B, kappa):
  robots:  list of cells (labeled)
  shelves: target positions dict {id: cell} + anonymous occupancy set
  kappa:   per-robot carrying: None | target-id str | "ANON"

Actions per robot per timestep: ("wait",) ("move", cell) ("lift",) ("drop",)

Rule table implemented exactly (R1 R2 S1 I1 I2 I3); S2 implied by R2.
This module is the single implementation of the rule table: unit tests,
scrambler, B4 baseline, and converters all go through it.
"""

from dataclasses import dataclass, field, replace
from typing import Dict, FrozenSet, List, Optional, Set, Tuple

from .instance import Cell, Instance

ANON = "ANON"

Action = Tuple  # ("wait",) | ("move", cell) | ("lift",) | ("drop",)


@dataclass(frozen=True)
class State:
    robots: Tuple[Cell, ...]
    target_pos: Tuple[Tuple[str, Cell], ...]  # sorted by id
    anon_occ: FrozenSet[Cell]
    kappa: Tuple[Optional[str], ...]  # None | target id | ANON

    def targets(self) -> Dict[str, Cell]:
        return dict(self.target_pos)

    def shelf_cells(self) -> Set[Cell]:
        return set(self.anon_occ) | {p for _, p in self.target_pos}

    def carried_shelves(self) -> Set[str]:
        return {k for k in self.kappa if k is not None and k != ANON}


def initial_state(ins: Instance) -> State:
    tpos = tuple(sorted((t.id, tuple(t.start)) for t in ins.targets))
    tcells = {tuple(t.start) for t in ins.targets}
    anon = frozenset(tuple(p) for p in ins.shelves if tuple(p) not in tcells)
    return State(
        robots=tuple(tuple(q) for q in ins.robots),
        target_pos=tpos,
        anon_occ=anon,
        kappa=tuple(None for _ in ins.robots),
    )


def is_goal(ins: Instance, s: State) -> bool:
    """Goal (design_final 2.2/Prop 3): every target grounded on an
    ELIGIBLE goal cell (set membership; singleton == old equality)."""
    carried = s.carried_shelves()
    pos = s.targets()
    for t in ins.targets:
        if t.id in carried:
            return False
        if pos.get(t.id) not in t.eligible_goals():
            return False
    return True


class TransitionError(ValueError):
    pass


def apply_joint_action(
    ins: Instance, s: State, actions: List[Action]
) -> State:
    """Validate and apply one joint action.  Raises TransitionError with the
    violated rule id if the transition is illegal (rules R1 R2 S1 I1 I2 I3 and
    per-robot preconditions of design.md 3.2)."""
    n = len(s.robots)
    if len(actions) != n:
        raise TransitionError(f"need {n} actions, got {len(actions)}")

    target_at: Dict[Cell, str] = {p: b for b, p in s.target_pos}
    grounded: Set[Cell] = set(s.anon_occ) | set(target_at)
    # remove carried target shelves from grounded view: a carried shelf is not
    # grounded.  kappa invariant: carrier cell == shelf cell for targets.
    carried_ids = s.carried_shelves()
    grounded_shelf_cells = set(s.anon_occ) | {
        p for b, p in s.target_pos if b not in carried_ids
    }

    new_robots: List[Cell] = list(s.robots)
    new_kappa: List[Optional[str]] = list(s.kappa)
    new_target_pos: Dict[str, Cell] = s.targets()
    new_anon: Set[Cell] = set(s.anon_occ)
    lifted_this_step: List[Cell] = []
    dropped_this_step: Set[Cell] = set()

    # ---- per-robot preconditions & effects (tentative) ----
    for i, act in enumerate(actions):
        q = s.robots[i]
        kind = act[0]
        if kind == "wait":
            continue
        elif kind == "move":
            v = tuple(act[1])
            if v not in ins.neighbors(q):
                raise TransitionError(f"robot {i}: move target {v} not neighbor of {q}")
            new_robots[i] = v
            if s.kappa[i] is not None:
                # carried shelf moves with carrier (S1 checked globally below)
                if s.kappa[i] == ANON:
                    pass  # anonymous carried shelf has no stored cell
                else:
                    new_target_pos[s.kappa[i]] = v
        elif kind == "lift":
            if s.kappa[i] is not None:
                raise TransitionError(f"robot {i}: lift while loaded")
            # I1: object must be grounded at step start
            if q not in grounded_shelf_cells:
                raise TransitionError(f"robot {i}: lift at {q}, no grounded shelf (I1)")
            lifted_this_step.append(q)
            if q in target_at and target_at[q] not in carried_ids:
                new_kappa[i] = target_at[q]
            else:
                new_kappa[i] = ANON
                new_anon.discard(q)
        elif kind == "drop":
            if s.kappa[i] is None:
                raise TransitionError(f"robot {i}: drop while free")
            new_kappa[i] = None
            dropped_this_step.add(q)
            if s.kappa[i] == ANON:
                new_anon.add(q)
            # target drop: position already tracked in new_target_pos
        else:
            raise TransitionError(f"robot {i}: unknown action {act!r}")

    # I1 atomicity: a shelf dropped this step may not also be lifted this step
    for cell in lifted_this_step:
        if cell in dropped_this_step:
            raise TransitionError(f"lift at {cell} of shelf dropped this step (I1)")
    # I2: one carrier per shelf — two lifts at same cell impossible via R1
    # (robots distinct at t), but two robots at same cell at t would be a
    # start-state violation; still guard duplicate lift targets:
    if len(lifted_this_step) != len(set(lifted_this_step)):
        raise TransitionError("two robots lift the same shelf (I2)")

    # ---- R1: vertex conflict on lower deck ----
    if len(set(new_robots)) != n:
        raise TransitionError("robot vertex conflict (R1)")
    # ---- R2: swap conflict on lower deck ----
    pos_of = {q: i for i, q in enumerate(s.robots)}
    for i, q_new in enumerate(new_robots):
        j = pos_of.get(q_new)
        if j is not None and j != i and new_robots[j] == s.robots[i]:
            raise TransitionError(f"robot swap conflict {i}<->{j} (R2)")

    # ---- S1: shelf vertex conflict on upper deck at t+1 ----
    upper_cells: List[Cell] = list(new_anon)
    new_carried = {k for k in new_kappa if k is not None and k != ANON}
    for b, p in new_target_pos.items():
        upper_cells.append(p)
    # anonymous carried shelves occupy their carrier's new cell:
    for i, k in enumerate(new_kappa):
        if k == ANON:
            upper_cells.append(new_robots[i])
    if len(upper_cells) != len(set(upper_cells)):
        raise TransitionError("shelf vertex conflict (S1)")

    # ---- loaded move destination upper-deck check is S1 (covered above);
    # I3 free robot may sit under grounded shelf: nothing to check.

    # ---- kappa invariant: carrier and carried target shelf co-located ----
    for i, k in enumerate(new_kappa):
        if k is not None and k != ANON:
            if new_target_pos[k] != new_robots[i]:
                raise TransitionError(f"kappa invariant broken for robot {i}")

    return State(
        robots=tuple(new_robots),
        target_pos=tuple(sorted(new_target_pos.items())),
        anon_occ=frozenset(new_anon),
        kappa=tuple(new_kappa),
    )


def legal_actions_for_robot(ins: Instance, s: State, i: int) -> List[Action]:
    """Enumerate per-robot local operator candidates (branch factor <= deg+3).
    Global legality still requires apply_joint_action."""
    q = s.robots[i]
    acts: List[Action] = [("wait",)]
    for v in ins.neighbors(q):
        acts.append(("move", v))
    carried_ids = s.carried_shelves()
    grounded_shelf_cells = set(s.anon_occ) | {
        p for b, p in s.target_pos if b not in carried_ids
    }
    if s.kappa[i] is None:
        if q in grounded_shelf_cells:
            acts.append(("lift",))
    else:
        acts.append(("drop",))
    return acts


def validate_plan(
    ins: Instance, plan: List[List[Action]], require_goal: bool = True
) -> Tuple[bool, List[str], State]:
    """Validate a full plan (list of joint actions) from the instance's initial
    state.  Returns (ok, errors, final_state)."""
    errors = ins.validate_static()
    if errors:
        return False, errors, None
    s = initial_state(ins)
    for t, joint in enumerate(plan):
        try:
            s = apply_joint_action(ins, s, joint)
        except TransitionError as e:
            errors.append(f"t={t}: {e}")
            return False, errors, s
    if require_goal and not is_goal(ins, s):
        errors.append("final state is not a goal state")
        return False, errors, s
    return True, errors, s


# ---- cost model (design.md 2.3) ----

def plan_cost(
    s0_ins: Instance,
    plan: List[List[Action]],
    alpha: float = 1.0,
    beta: float = 1.0,
    gamma: float = 1.0,
    delta: float = 1.0,
) -> Dict[str, float]:
    """Weighted action cost + executed makespan.  Assumes plan is valid."""
    s = initial_state(s0_ins)
    loaded_moves = free_moves = liftdrops = anon_moves = 0
    shelf_switches = 0
    reversals = 0
    prev_pos = list(s.robots)      # position at t-1
    prev_prev = list(s.robots)     # position at t-2
    executed_makespan = None       # FIRST time the goal holds
    lift_counts = {}       # shelf identity -> number of lifts so far
    anon_id_at = {}        # cell -> anon identity (custody chain)
    carrier_custody = {}   # robot i -> identity currently carried
    next_anon_id = [0]
    for t, joint in enumerate(plan):
        for i, act in enumerate(joint):
            if act[0] == "lift":
                cell = s.robots[i]
                tgt = {p: b for b, p in s.target_pos}
                if cell in tgt:
                    key = ("tgt", tgt[cell])
                else:
                    if cell not in anon_id_at:
                        anon_id_at[cell] = next_anon_id[0]
                        next_anon_id[0] += 1
                    key = ("anon", anon_id_at.pop(cell))
                carrier_custody[i] = key
                n = lift_counts.get(key, 0)
                if n >= 1:
                    shelf_switches += 1
                lift_counts[key] = n + 1
            if act[0] == "move":
                if s.kappa[i] is None:
                    free_moves += 1
                else:
                    loaded_moves += 1
                    if s.kappa[i] == ANON:
                        anon_moves += 1
            elif act[0] in ("lift", "drop"):
                liftdrops += 1
                if act[0] == "drop":
                    key = carrier_custody.pop(i, None)
                    if key is not None and key[0] == "anon":
                        anon_id_at[s.robots[i]] = key[1]
        s = apply_joint_action(s0_ins, s, joint)
        # oscillation metric (design 8.3 addition, round-2 P2-13a):
        # immediate A->B->A flips in the position history — moved at t-1
        # AND back on the t-2 position now.  Wait in between doesn't count.
        if t >= 1:
            for i, q in enumerate(s.robots):
                if prev_pos[i] != prev_prev[i] and q == prev_prev[i]:
                    reversals += 1
        prev_prev = prev_pos
        prev_pos = list(s.robots)
        if executed_makespan is None and is_goal(s0_ins, s):
            executed_makespan = t + 1
            # makespan is the FIRST time the goal holds; the replay keeps
            # going so metrics cover the WHOLE plan (matches the long-
            # standing comment; the old `break` truncated counters and made
            # zero-target fixtures degenerate)
    if executed_makespan is None:
        executed_makespan = len(plan)
    # shelf switches (design 8.3): re-lifts, i.e. lift events beyond each
    # shelf's first lift.  Track identity for targets; anonymous shelves are
    # tracked by cell chain-of-custody within this replay.
    return {
        "executed_makespan": executed_makespan,
        "loaded_moves": loaded_moves,
        "free_moves": free_moves,
        "lift_drop": liftdrops,
        "anon_moves": anon_moves,
        "shelf_switches": shelf_switches,
        "reversals": reversals,
        "robot_utilization": (
            loaded_moves / (len(s0_ins.robots) * executed_makespan)
            if executed_makespan > 0 else 0.0
        ),
        "weighted_soc": alpha * loaded_moves
        + beta * free_moves
        + gamma * liftdrops
        + delta * anon_moves,
    }
