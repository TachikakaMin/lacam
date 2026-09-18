#!/usr/bin/env python3
"""Independent validator for demo plan JSONs (pyramid and column).

Checks over the whole tape:
  1. paths: unit moves only, |dh| <= 1 wrt terrain at that moment
  2. no vertex / edge collisions between agents
  3. picks at depot cells, carry 0->1
  4. placements: stacked +1, cell agent-free, acting agent adjacent at h-1,
     its carry 1->0
  5. removals: top block -1, cell agent-free, acting agent adjacent at new h,
     its carry 0->1
  6. deposits at depot cells, carry 1->0
  7. conservation: picks + removals == placements + deposits;
     final heights match target (final_heights, or pyramid shape)

usage: validate_plan.py [plan.json ...]  (default: plan.json)
"""
import json
import sys
from pathlib import Path

NBRS = ((-1, 0), (1, 0), (0, -1), (0, 1))


def validate(path: Path) -> int:
    plan = json.loads(path.read_text())
    ROWS, COLS, T = plan["rows"], plan["cols"], plan["T"]
    depot = {tuple(d) for d in plan["depot"]}
    events = sorted(plan["terrain"], key=lambda e: e["t"])
    picks = plan.get("picks", [])
    deposits = plan.get("deposits", [])
    agents = plan["agents"]
    N = len(agents)

    errors = 0

    def err(msg):
        nonlocal errors
        errors += 1
        print(f"ERROR [{path.name}]: {msg}")

    for a in agents:
        if len(a["path"]) != T + 1 or len(a["carry"]) != T + 1:
            err(f"bad history length for {a['name']}")
            return 1

    h = [[0] * COLS for _ in range(ROWS)]
    n_place = n_remove = 0
    ev_idx = 0
    for t in range(T + 1):
        pos = [tuple(a["path"][t]) for a in agents]
        if len(set(pos)) != N:
            err(f"vertex collision at t={t}: {pos}")
        if t > 0:
            prev = [tuple(a["path"][t - 1]) for a in agents]
            for i in range(N):
                (r0, c0), (r1, c1) = prev[i], pos[i]
                if abs(r0 - r1) + abs(c0 - c1) > 1:
                    err(f"agent {i} non-unit move at t={t}")
                if abs(h[r1][c1] - h[r0][c0]) > 1:
                    err(f"agent {i} height jump at t={t}: {h[r0][c0]}->{h[r1][c1]}")
            for i in range(N):
                for j in range(i + 1, N):
                    if prev[i] == pos[j] and prev[j] == pos[i]:
                        err(f"edge collision {i},{j} at t={t}")
        # group multi-cell brick events (same t + agent + "bw" marker)
        group_start = ev_idx
        brick_groups = {}
        j = ev_idx
        while j < len(events) and events[j]["t"] == t:
            e2 = events[j]
            if "bw" in e2:
                brick_groups.setdefault(e2.get("agent"), []).append(e2)
            j += 1
        checked_groups = set()
        while ev_idx < len(events) and events[ev_idx]["t"] == t:
            ev = events[ev_idx]
            r, c, hh = ev["r"], ev["c"], ev["h"]
            ai = ev.get("agent")
            actor = agents[ai] if ai is not None else None
            if (r, c) in pos:
                err(f"terrain event on occupied cell t={t} ({r},{c})")
            if "bw" in ev and hh > h[r][c]:  # LEGO brick cell: validate per group
                if ai not in checked_groups:
                    n_place += 1  # one pick = one brick = one placement
                    checked_groups.add(ai)
                    grp = brick_groups[ai]
                    supported = any(h[e2["r"]][e2["c"]] == e2["h"] - 1 for e2 in grp)
                    adh = supported or any(
                        0 <= e2["r"] + dr < ROWS and 0 <= e2["c"] + dc < COLS
                        and h[e2["r"] + dr][e2["c"] + dc] >= e2["h"]
                        for e2 in grp for dr, dc in NBRS)
                    if not adh:
                        err(f"brick without stud support or adhesion: t={t}")
                    if actor is not None:
                        ar, ac = actor["path"][t]
                        near = any(abs(ar - e2["r"]) + abs(ac - e2["c"]) == 1
                                   and h[ar][ac] in (e2["h"] - 1, e2["h"])
                                   for e2 in grp)  # strict: same or one below
                        if not near:
                            err(f"brick placer stand invalid: t={t}")
                        if not (actor["carry"][t - 1] == 1 and actor["carry"][t] == 0):
                            err(f"brick placer carry not 1->0 at t={t}")
                h[r][c] = hh
                ev_idx += 1
                continue
            if hh > h[r][c]:  # placement (hh == old+1: stacked; hh > old+1: bridged)
                n_place += 1
                bridged = hh > h[r][c] + 1
                if bridged:
                    # side adhesion: a face-adjacent block at the same z
                    adh = any(0 <= r + dr < ROWS and 0 <= c + dc < COLS
                              and h[r + dr][c + dc] >= hh
                              for dr, dc in NBRS)
                    if not adh:
                        err(f"bridged placement without side adhesion: t={t} ({r},{c})")
                if actor is not None:
                    ar, ac = actor["path"][t]
                    ok_stand = (abs(ar - r) + abs(ac - c) == 1
                                and h[ar][ac] in ((hh, hh + 1) if bridged else (hh - 1, hh)))
                    if not ok_stand:
                        err(f"placer stand invalid: t={t} ({r},{c})")
                    if not (actor["carry"][t - 1] == 1 and actor["carry"][t] == 0):
                        err(f"placer carry not 1->0 at t={t}")
            elif hh == h[r][c] - 1:  # removal
                n_remove += 1
                if actor is not None:
                    ar, ac = actor["path"][t]
                    if abs(ar - r) + abs(ac - c) != 1 or h[ar][ac] not in (hh, hh + 1):
                        err(f"remover stand invalid: t={t} ({r},{c})")
                    if not (actor["carry"][t - 1] == 0 and actor["carry"][t] == 1):
                        err(f"remover carry not 0->1 at t={t}")
            else:
                err(f"non-unit terrain event t={t} ({r},{c}): {h[r][c]}->{hh}")
            h[r][c] = hh
            ev_idx += 1

    for p in picks:
        if (p["r"], p["c"]) not in depot:
            err(f"pick not at depot: {p}")
        a = agents[p["agent"]]
        if tuple(a["path"][p["t"]]) != (p["r"], p["c"]) or a["carry"][p["t"]] != 1:
            err(f"pick inconsistent: {p}")
    for d in deposits:
        if (d["r"], d["c"]) not in depot:
            err(f"deposit not at depot: {d}")
        a = agents[d["agent"]]
        if tuple(a["path"][d["t"]]) != (d["r"], d["c"]) or a["carry"][d["t"]] != 0:
            err(f"deposit inconsistent: {d}")

    if len(picks) + n_remove != n_place + len(deposits):
        err(f"block conservation: picks {len(picks)} + removals {n_remove} "
            f"!= placements {n_place} + deposits {len(deposits)}")
    for a in agents:
        if a["carry"][T] != 0:
            err(f"{a['name']} still carrying at the end")

    if "final_heights" in plan:
        expect = plan["final_heights"]
    elif "pyramid" in plan:
        pyr = plan["pyramid"]
        expect = [[0] * COLS for _ in range(ROWS)]
        for lvl in range(1, pyr["levels"] + 1):
            inset = lvl - 1
            side = pyr["base"] - 2 * inset
            if side <= 0:
                break
            for r in range(pyr["r0"] + inset, pyr["r0"] + inset + side):
                for c in range(pyr["c0"] + inset, pyr["c0"] + inset + side):
                    expect[r][c] = lvl
    else:
        expect = None
    if expect is not None and h != expect:
        err("final heights do not match target")

    print(f"[{path.name}] T={T}, agents={N}, placements={n_place}, removals={n_remove}, "
          f"picks={len(picks)}, deposits={len(deposits)}")
    print(f"[{path.name}] RESULT:", "FAIL" if errors else "PASS")
    return 1 if errors else 0


if __name__ == "__main__":
    args = sys.argv[1:] or ["plan.json"]
    base = Path(__file__).parent
    rc = 0
    for name in args:
        p = Path(name)
        if not p.exists():
            p = base / name
        rc |= validate(p)
    sys.exit(rc)
