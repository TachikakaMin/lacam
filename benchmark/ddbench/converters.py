"""Converters from the dd-lacam YAML instance format to baseline inputs.

CREST (github.com/ChristinaTan0704/CREST):
  map:  movingai header; '@' marks SHELF-occupied cells (Executor turns every
        '@' cell not listed as a task pickup into an anonymous goal=start
        task), '.' free.  True walls are not modeled by the executor, so only
        wall-free instances should be exported.
  scen: first line = number of target tasks; task lines 'row col grow gcol';
        then a 'version 1' line and one Nathan-style line per agent where
        items 5,6 are the agent start 'row col'.

MAWR / NAT-CBS (github.com/CRL-Technion/wh-rearrangement):
  map:  first line 'rows cols'; '#' static obstacle, '@' blocked, '.' free.
  scen: 'num_agents num_obstacles num_tasks'; agent lines 'col row'
        (Location: operator>> fills y then x with x=row); obstacle lines
        'pcol prow dcol drow'; tasks = obstacles with pickup != delivery.
"""

from pathlib import Path
from typing import Tuple

from .instance import Instance


class ConversionError(ValueError):
    pass


def _match_anon_goals(ins: Instance):
    """Anonymous shelf start -> goal cell for full-goal-layout baselines.

    Uses the exact witness pairing recorded by the scrambler when available
    (guaranteed realizable); otherwise anonymous shelves stay put.
    Returns dict start_cell -> goal_cell.
    """
    target_starts = {tuple(t.start) for t in ins.targets}
    anon_starts = [tuple(p) for p in ins.shelves if tuple(p) not in target_starts]
    if ins.anon_goals is None:
        return {p: p for p in anon_starts}
    match = {tuple(s): tuple(g) for s, g in ins.anon_goals}
    if set(match) != set(anon_starts):
        raise ConversionError(
            "anon_goals starts do not cover the anonymous shelf cells"
        )
    return match


def to_crest(ins: Instance, map_path, scen_path) -> None:
    if any(any(row) for row in ins.grid):
        raise ConversionError(
            "CREST executor does not model walls; instance must be wall-free"
        )
    h, w = ins.height, ins.width
    shelf_cells = {tuple(p) for p in ins.shelves}
    lines = ["type octile", f"height {h}", f"width {w}", "map"]
    for r in range(h):
        lines.append(
            "".join("@" if (r, c) in shelf_cells else "." for c in range(w))
        )
    Path(map_path).write_text("\n".join(lines) + "\n")

    anon_match = _match_anon_goals(ins)
    tasks = [(tuple(t.start), tuple(t.goal)) for t in ins.targets]
    # anonymous shelves that must relocate become explicit tasks; the ones
    # staying put are covered by CREST's implicit goal=start @-cell handling.
    tasks += [(s, g) for s, g in sorted(anon_match.items()) if s != g]
    out = [str(len(tasks))]
    for (sr, sc), (gr, gc) in tasks:
        out.append(f"{sr} {sc} {gr} {gc}")
    out.append("version 1")
    for (r, c) in ins.robots:
        out.append(f"0\tr\t{h}\t{w}\t{r}\t{c}\t0\t0\t0")
    Path(scen_path).write_text("\n".join(out) + "\n")


def to_mawr(ins: Instance, map_path, scen_path) -> None:
    h, w = ins.height, ins.width
    lines = [f"{h} {w}"]
    for r in range(h):
        lines.append("".join("#" if ins.grid[r][c] else "." for c in range(w)))
    Path(map_path).write_text("\n".join(lines) + "\n")

    target_start = {tuple(t.start): t for t in ins.targets}
    anon_match = _match_anon_goals(ins)

    def goal_of(p):
        t = target_start.get(p)
        if t is not None:
            return tuple(t.goal)
        return anon_match.get(p, p)

    n_tasks = sum(1 for p in ins.shelves if goal_of(tuple(p)) != tuple(p))
    out = [f"{len(ins.robots)} {len(ins.shelves)} {n_tasks}"]
    for (r, c) in ins.robots:
        out.append(f"{c} {r}")
    for p in ins.shelves:
        p = tuple(p)
        gr, gc = goal_of(p)
        out.append(f"{p[1]} {p[0]} {gc} {gr}")
    Path(scen_path).write_text("\n".join(out) + "\n")
