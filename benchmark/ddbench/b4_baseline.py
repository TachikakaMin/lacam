"""B4 baseline: single-robot sequential simulation (design.md 8.1, Theorem 1).

Robot 0 executes a sequential pebble-motion style plan: for each target shelf
(nearest-goal first), compute an upper-deck path, recursively clear blocking
shelves to parking cells, then carry the target along the path.  All other
robots wait.  Complete only on easy instances — reported success rate is the
point (sanity + throughput lower bound).

Every emitted joint action goes through the validator (generator proposes,
validator accepts), so any bug fails loudly rather than corrupting metrics.
"""

from collections import deque
from typing import Dict, List, Optional, Set, Tuple

from .instance import Cell, Instance
from .validator import (
    ANON,
    State,
    TransitionError,
    apply_joint_action,
    initial_state,
    is_goal,
)


class B4Failure(Exception):
    pass


def _bfs_path(
    ins: Instance,
    src: Cell,
    dst: Cell,
    blocked: Set[Cell],
) -> Optional[List[Cell]]:
    """Shortest path src->dst on the grid avoiding walls and `blocked` cells
    (src itself may be in blocked)."""
    if src == dst:
        return [src]
    if dst in blocked:
        return None
    prev: Dict[Cell, Cell] = {}
    dq = deque([src])
    seen = {src}
    while dq:
        u = dq.popleft()
        for v in ins.neighbors(u):
            if v in seen or v in blocked:
                continue
            seen.add(v)
            prev[v] = u
            if v == dst:
                path = [v]
                while path[-1] != src:
                    path.append(prev[path[-1]])
                return list(reversed(path))
            dq.append(v)
    return None


class B4Runner:
    def __init__(self, ins: Instance, max_steps: int = 200000):
        self.ins = ins
        self.s = initial_state(ins)
        self.plan: List[List] = []
        self.max_steps = max_steps
        self.n = len(ins.robots)

    # ---- emit one joint action where only robot `idx` acts ----
    def _step(self, act, idx: int = 0) -> None:
        if len(self.plan) >= self.max_steps:
            raise B4Failure("max steps exceeded")
        joint = [("wait",)] * self.n
        joint[idx] = act
        try:
            self.s = apply_joint_action(self.ins, self.s, joint)
        except TransitionError as e:
            raise B4Failure(f"illegal emitted action {act} (robot {idx}): {e}") from e
        self.plan.append(joint)

    def _shoo(self, v: Cell, depth: int = 0) -> None:
        """If an idle robot sits on cell v, move it one step aside
        (recursively shooing its own blockers if needed)."""
        if depth > self.n + 2:
            raise B4Failure(f"shoo recursion too deep at {v}")
        for j in range(1, self.n):
            if self.s.robots[j] == v:
                occupied = set(self.s.robots)
                nbrs = self.ins.neighbors(v)
                for u in nbrs:
                    if u not in occupied:
                        self._step(("move", u), idx=j)
                        return
                # all neighbors occupied: shoo a neighboring robot first
                for u in nbrs:
                    if u != self._robot():
                        self._shoo(u, depth + 1)
                        self._step(("move", u), idx=j)
                        return
                raise B4Failure(f"idle robot at {v} cannot be shooed")

    def _robot(self) -> Cell:
        return self.s.robots[0]

    def _upper(self) -> Set[Cell]:
        return self.s.shelf_cells()

    def _other_robots(self) -> Set[Cell]:
        return set(self.s.robots[1:])

    def _free_move_to(self, dst: Cell) -> None:
        """Free robot travel: walls block; idle robots get shooed aside."""
        path = _bfs_path(self.ins, self._robot(), dst, set())
        if path is None:
            raise B4Failure(f"free travel to {dst} impossible")
        for v in path[1:]:
            self._shoo(v)
            self._step(("move", v))

    def _carry_shelf(self, shelf_cell: Cell, path: List[Cell]) -> None:
        """Lift shelf at shelf_cell (robot must travel there first) and carry
        it along `path` (upper-deck cells must be clear), then drop."""
        self._free_move_to(shelf_cell)
        self._step(("lift",))
        for v in path[1:]:
            self._shoo(v)
            self._step(("move", v))
        self._step(("drop",))

    def _clear_cell(self, cell: Cell, protected: Set[Cell], depth: int = 0) -> None:
        """Relocate the shelf at `cell` to some parking cell not in
        `protected`.  Re-computes the relocation path each round so it stays
        correct when deeper recursions move shelves around."""
        if depth > len(self.ins.shelves) + 4:
            raise B4Failure("clearing recursion too deep")
        for _round in range(len(self.ins.shelves) + 4):
            upper = self._upper()
            if cell not in upper:
                return  # already cleared by a deeper recursion
            parking_blocked = upper | protected
            # BFS over walls only: nearest legal parking spot
            prev: Dict[Cell, Cell] = {}
            dq = deque([cell])
            seen = {cell}
            target_cell: Optional[Cell] = None
            while dq:
                u = dq.popleft()
                for v in self.ins.neighbors(u):
                    if v in seen:
                        continue
                    seen.add(v)
                    prev[v] = u
                    if v not in parking_blocked:
                        target_cell = v
                        dq.clear()
                        break
                    dq.append(v)
            if target_cell is None:
                raise B4Failure(f"no parking cell reachable from {cell}")
            path = [target_cell]
            while path[-1] != cell:
                path.append(prev[path[-1]])
            path = list(reversed(path))
            blockers = [c for c in path[1:] if c in upper]
            if not blockers:
                self._carry_shelf(cell, path)
                return
            # clear the blocker closest to the parking end first
            self._clear_cell(blockers[-1], protected | set(path), depth + 1)
        raise B4Failure(f"clearing at {cell} did not converge")

    def solve(self) -> List[List]:
        ins = self.ins
        # serve targets nearest-goal-first (static order); multiple passes
        # because clearing may displace already-completed targets.
        order = sorted(
            ins.targets,
            key=lambda t: abs(t.start[0] - t.goal[0]) + abs(t.start[1] - t.goal[1]),
        )
        for _pass in range(6):
            pos_all = dict(self.s.target_pos)
            pending = [t for t in order if pos_all[t.id] != tuple(t.goal)]
            if not pending:
                break
            for t in pending:
                self._serve_target(t)
        if not is_goal(ins, self.s):
            raise B4Failure("plan finished but goal not reached")
        return self.plan

    def _serve_target(self, t) -> None:
        ins = self.ins
        for _round in range(len(ins.shelves) + 5):
            pos = dict(self.s.target_pos)[t.id]
            goal = tuple(t.goal)
            if pos == goal:
                return
            upper = self._upper()
            # prefer a shelf-free path; else walls-only path + clearing
            path = _bfs_path(ins, pos, goal, upper - {pos})
            if path is None:
                path = _bfs_path(ins, pos, goal, set())
                if path is None:
                    raise B4Failure(f"target {t.id}: no wall-free path")
                # clear first blocker on the path, then retry
                protected = set(path)
                blocker = next(c for c in path[1:] if c in upper)
                self._clear_cell(blocker, protected)
                continue
            self._carry_shelf(pos, path)
        raise B4Failure(f"target {t.id}: clearing did not converge")


def solve_b4(ins: Instance, max_steps: int = 200000) -> List[List]:
    """Returns a validated plan or raises B4Failure."""
    return B4Runner(ins, max_steps=max_steps).solve()
