#!/usr/bin/env python3
"""Offline planner: build a 6-high column with a temporary staircase, then
dismantle the staircase and return the blocks to the depot.

Task assignment is done by LaCAM-TAPF itself: every round each agent gets a
*set* of candidate goals (potentialGoals) -- all valid stand cells of all
currently-ready placements/removals, or all depot cells -- and the solver's
integrated assignment picks the optimal matching. Python only classifies
agent states, recovers task identity from the solver's assignment, and
executes until the next completed action (pick/place/remove/deposit, each
occupying one timestep), then replans globally.

Construction order comes from dependency rules (no reservations, no greedy
claiming):
  place (x,l): support below; ground level freely; l>=2 never above a deeper
               neighbor (staircase stays monotone)
  remove (x,h): x is at h; every deeper scaffold neighbor already <= h
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
from tapf_schedule_io import load_schedule  # noqa: E402

Coord = Tuple[int, int]

# ---------------------------------------------------------------- config
ROWS, COLS = 14, 22
COL_CELL: Coord = (7, 7)
COL_H = 6
RAMPS: List[List[Coord]] = []
SCAFFOLD_CELLS: List[Coord] = []
T_BUILD: Dict[Coord, int] = {}
T_FINAL: Dict[Coord, int] = {}
LINE_CELLS: set = set()
DECK_CELLS: set = set()  # cells whose bottom slab block is side-adhered (floats)
DECK_LEVEL: Dict[Coord, int] = {}  # cell -> z of its floating bottom block
DECK_Z = 0  # legacy: scenes with a single deck level set both
ORDER: Dict[Coord, int] = {}  # optional build-order field (default: T_BUILD)
FORBIDDEN_CELLS: set = set()  # cells agents must never stand on (sealed interior)
COLORS: Dict[Coord, str] = {}  # optional cell -> "#rrggbb" for the viewer
COLORS3: Dict[Tuple[int, int, int], str] = {}  # optional (r, c, h) -> color
SUSPENDERS: List[Dict[str, int]] = []  # viewer-only vertical cable drop lines
STRUTS: List[Dict[str, int]] = []  # viewer-only horizontal tower struts
POCKET_CELLS: set = set()  # auto passing pockets (corridor capacity hints)
POCKET_EVERY_HINT = 0  # scene hint: override AUTO_POCKET_EVERY (0 = default)
DEPOT = [(r, 20) for r in range(3, 11)]
AGENT_STARTS = [(3, 18), (4, 18), (5, 18), (6, 18), (7, 18), (8, 18)]


def set_agents(n: int) -> None:
    global AGENT_STARTS
    cols = [18, 17, 16, 15]
    AGENT_STARTS = [(3 + i % 6, cols[i // 6]) for i in range(n)]
BINARY = REPO / "build" / "tapf_benchmark"
WORK = Path(__file__).resolve().parent / "work_column"
OUT_JSON = Path(__file__).resolve().parent / "plan_column.json"
OUT_JS = Path(__file__).resolve().parent / "plan_column.js"
TIME_LIMIT = float(os.environ.get("SOLVER_TIME", "2.0"))
MAX_ROUNDS = 15000
MAX_STEPS = 50000
INF = 10 ** 9
CLIMB_COST = 2
# assignment cost offsets (solver distance units; step=1, climb=CLIMB_COST)
HOLD_COST = 10000   # hold-as-goal: feasibility fallback, avoided if possible
STICKY_BONUS = int(os.environ.get("STICKY_BONUS", "0"))  # 0 = auto per scene
PRIO_W = 3          # weight per missing urgency level (deeper chain = cheaper)
# work dir policy: 0 = reuse a single "current" dir; N>0 = keep last N rounds
KEEP_ROUNDS = int(os.environ.get("KEEP_ROUNDS", "0"))
# parallelism knobs: auto passing pockets on long scaffold corridors, and the
# single-file task-segment length on those corridors
AUTO_POCKETS = os.environ.get("AUTO_POCKETS", "1") != "0"
AUTO_POCKET_EVERY = int(os.environ.get("AUTO_POCKET_EVERY", "0"))  # 0 = auto
SCAFFOLD_MIN = int(os.environ.get("SCAFFOLD_MIN", "12"))
AUTO_POCKET_MODE = os.environ.get("AUTO_POCKET_MODE", "")  # "" = auto
AUTO_POCKET_EVERY_SET = "AUTO_POCKET_EVERY" in os.environ
SCAFFOLD_SEG = int(os.environ.get("SCAFFOLD_SEG", "4"))
# small-experiment budget: stop after N wall seconds and report partial
# throughput (actions per timestep) instead of failing
MAX_WALL_SEC = float(os.environ.get("MAX_WALL_SEC", "0"))
TRACE_A2 = os.environ.get("TRACE_A2", "")
THROTTLE_AT = int(os.environ.get("THROTTLE_AT", "30"))
THROTTLE_HOLD = int(os.environ.get("THROTTLE_HOLD", "150"))
# solver knobs (see tools/tapf_benchmark.cpp usage)
SOLVER_ANYTIME = os.environ.get("SOLVER_ANYTIME", "0")
SOLVER_MODE = os.environ.get("SOLVER_MODE", "dfs")
SOLVER_FOCAL_W = os.environ.get("SOLVER_FOCAL_W", "1.5")
SOLVER_TIE = os.environ.get("SOLVER_TIE", "h")

NBRS = ((-1, 0), (1, 0), (0, -1), (0, 1))


def configure(col_h: int, helper: bool) -> None:
    """Main ramp east of the column; optional parallel helper lane one row
    north, one level lower (its purpose is faster demolition of the main
    ramp, per the study)."""
    global COL_H, RAMPS, SCAFFOLD_CELLS, T_BUILD, T_FINAL, LINE_CELLS
    COL_H = col_h
    main = [(7, 7 + 1 + k) for k in range(col_h - 1)]        # targets H-1..1
    RAMPS = [main]
    T_BUILD = {COL_CELL: col_h}
    for k, cell in enumerate(main):
        T_BUILD[cell] = col_h - 1 - k
    if helper:
        lane = []
        for k, cell in enumerate(main):
            tgt = T_BUILD[cell] - 1                            # one lower
            if tgt >= 1:
                hc = (6, cell[1])                              # parallel, north
                lane.append(hc)
                T_BUILD[hc] = tgt
        if lane:
            RAMPS.append(lane)
    SCAFFOLD_CELLS = [c for ramp in RAMPS for c in ramp]
    LINE_CELLS = set(SCAFFOLD_CELLS) | {COL_CELL}
    T_FINAL = {COL_CELL: col_h}


def configure_bridge() -> None:
    """Arch bridge: two 4-high towers with staircase scaffolds, deck built by
    side-adhesion bridging at z=4 from both ends, then stairs dismantled."""
    global ROWS, COLS, COL_H, COL_CELL, RAMPS, SCAFFOLD_CELLS
    global T_BUILD, T_FINAL, LINE_CELLS, DECK_CELLS, DECK_Z, DEPOT, AGENT_STARTS
    ROWS, COLS = 14, 22
    tower_a, tower_b = (7, 6), (7, 12)
    deck = [(7, 7), (7, 8), (7, 9), (7, 10), (7, 11)]
    ramp_a = [(7, 5), (7, 4), (7, 3)]     # west of tower A, targets 3,2,1
    ramp_b = [(7, 13), (7, 14), (7, 15)]  # east of tower B, targets 3,2,1
    DECK_Z = 4
    T_BUILD = {tower_a: 4, tower_b: 4}
    for ramp in (ramp_a, ramp_b):
        for k, cell in enumerate(ramp):
            T_BUILD[cell] = 3 - k
    for c in deck:
        T_BUILD[c] = DECK_Z
    T_FINAL = {tower_a: 4, tower_b: 4}
    for c in deck:
        T_FINAL[c] = DECK_Z
    COL_CELL = tower_a
    COL_H = 4
    RAMPS = [ramp_a, ramp_b]
    SCAFFOLD_CELLS = ramp_a + ramp_b
    LINE_CELLS = set(SCAFFOLD_CELLS) | {tower_a, tower_b}
    DECK_CELLS = set(deck)
    DEPOT = [(r, 20) for r in range(3, 11)]
    AGENT_STARTS = [(3 + i, 18) for i in range(8)]


def configure_temple() -> None:
    """Parthenon-style temple: outer stone band (h1), wall ring (z1..4) with a
    south door, roof slab at z=5 (wall caps + side-adhesion deck over the
    hollow interior and the door lintel), one staircase scaffold on the band.
    """
    global ROWS, COLS, COL_H, COL_CELL, RAMPS, SCAFFOLD_CELLS
    global T_BUILD, T_FINAL, LINE_CELLS, DECK_CELLS, DECK_Z, ORDER
    global FORBIDDEN_CELLS, DEPOT, AGENT_STARTS
    ROWS, COLS = 15, 24
    DECK_Z = 5
    T_BUILD, T_FINAL, ORDER = {}, {}, {}

    # wall ring rows 4..10 x cols 6..14, door at south center
    wall_rows, wall_cols = range(4, 11), range(6, 15)
    door = (10, 10)
    walls = []
    for r in wall_rows:
        for c in wall_cols:
            if (r in (4, 10) or c in (6, 14)) and (r, c) != door:
                walls.append((r, c))
    for w in walls:
        T_BUILD[w] = 5
        T_FINAL[w] = 5

    # outer band ring (one cell outside the walls), height 1, final
    band = []
    for r in range(3, 12):
        for c in range(5, 16):
            if (r in (3, 11) or c in (5, 15)) and (r, c) != door:
                band.append((r, c))
    for b in band:
        T_BUILD[b] = 1
        T_FINAL[b] = 1

    # staircase scaffolds on the band: east (roof access) and north (closing
    # the wall ring at the convergence cell), both stripped afterwards
    ramp_e = [(6, 15), (7, 15), (8, 15)]
    for cell, tgt in zip(ramp_e, (4, 3, 2)):
        T_BUILD[cell] = tgt
        T_FINAL[cell] = 1  # band block below stays
    ramp_n = [(3, 10), (3, 9), (3, 8)]
    for cell, tgt in zip(ramp_n, (4, 3, 2)):
        T_BUILD[cell] = tgt
        T_FINAL[cell] = 1

    # roof deck: hollow interior + door lintel (side adhesion at z=5)
    interior = [(r, c) for r in range(5, 10) for c in range(7, 14)]
    DECK_CELLS = set(interior) | {door}
    for c in DECK_CELLS:
        T_BUILD[c] = DECK_Z
        T_FINAL[c] = DECK_Z

    # wall build order: the ring minus the door is an open chain; waves start
    # at both door-adjacent endpoints and converge at the north-center cell,
    # which is served by the north closing ramp
    chain = []
    cur, prev = (10, 9), door
    while True:
        chain.append(cur)
        nxt = None
        for dr, dc in NBRS:
            n = (cur[0] + dr, cur[1] + dc)
            if n in walls and n != prev and n not in chain:
                nxt = n
                break
        if nxt is None:
            break
        prev, cur = cur, nxt
    L = len(chain)
    for i, w in enumerate(chain):
        ORDER[w] = 899 - min(i, L - 1 - i)

    # band ring: single-direction chain (closed loop is fine at height 1:
    # ground stands are always available outside)
    band_seed = (3, 15)
    ring2, cur, prev = [], band_seed, None
    while True:
        ring2.append(cur)
        nxt = None
        for dr, dc in NBRS:
            n = (cur[0] + dr, cur[1] + dc)
            if n in band and n != prev and n not in ring2:
                nxt = n
                break
        if nxt is None:
            break
        prev, cur = cur, nxt
    for i, b in enumerate(ring2):
        ORDER[b] = 800 - i

    # ramps: explicit order, deepest step (next to the wall) first, so the
    # staircase always ascends toward the wall (overrides band-chain order)
    for ramp in (ramp_e, ramp_n):
        for j, cell in enumerate(ramp):
            ORDER[cell] = 820 - j

    FORBIDDEN_CELLS = set(interior)  # never stand inside (sealed by the roof)
    COL_CELL = walls[0]
    COL_H = 5
    RAMPS = [ramp_e, ramp_n]
    SCAFFOLD_CELLS = list(ramp_e) + list(ramp_n)
    LINE_CELLS = set(walls) | set(band) | set(ramp_e) | set(ramp_n)
    DEPOT = [(r, 22) for r in range(4, 12)]
    AGENT_STARTS = [(3 + i % 5, 20 - i // 5)
                    for i in range(int(os.environ.get("N_AGENTS", "16")))]


def configure_colonnade() -> None:
    """Colonnade temple: 14 free-standing pillars (z1..4 + cap z5) carrying a
    floating roof slab; gaps between pillars are open (deck at z=5 with air
    below). Construction uses an ancient-style scaffold ring: a height-4 wall
    around the whole temple with stair tails, hand-over-hand laddering with
    the pillars; afterwards it is stripped down to the stone base band (h1).
    """
    global ROWS, COLS, COL_H, COL_CELL, RAMPS, SCAFFOLD_CELLS
    global T_BUILD, T_FINAL, LINE_CELLS, DECK_CELLS, DECK_Z, ORDER
    global FORBIDDEN_CELLS, DEPOT, AGENT_STARTS
    ROWS, COLS = 15, 24
    DECK_Z = 5
    T_BUILD, T_FINAL, ORDER = {}, {}, {}

    # colonnade ring rows 4..10 x cols 6..14; pillars on alternating cells
    ring = []
    cur, prev = (4, 6), None
    ring_cells = {(r, c) for r in range(4, 11) for c in range(6, 15)
                  if r in (4, 10) or c in (6, 14)}
    while True:
        ring.append(cur)
        nxt = None
        for dr, dc in NBRS:
            n = (cur[0] + dr, cur[1] + dc)
            if n in ring_cells and n != prev and n not in ring:
                nxt = n
                break
        if nxt is None:
            break
        prev, cur = cur, nxt
    pillars = [cell for i, cell in enumerate(ring) if i % 2 == 0]
    gaps = [cell for i, cell in enumerate(ring) if i % 2 == 1]
    for p in pillars:
        T_BUILD[p] = 5
        T_FINAL[p] = 5

    # scaffold ring: uniform height-4 wall (stone base h1 remains), built as
    # FOUR directional waves (one per side), each arc ending at a junction
    # cell served by its own perpendicular stair tail
    scaff_cells = {(r, c) for r in range(3, 12) for c in range(5, 16)
                   if r in (3, 11) or c in (5, 15)}
    ring2 = []
    cur, prev = (11, 10), None
    while True:
        ring2.append(cur)
        nxt = None
        for dr, dc in NBRS:
            n = (cur[0] + dr, cur[1] + dc)
            if n in scaff_cells and n != prev and n not in ring2:
                nxt = n
                break
        if nxt is None:
            break
        prev, cur = cur, nxt
    for cell in ring2:
        T_BUILD[cell] = 4
        T_FINAL[cell] = 1  # stone base band remains

    junctions = [(11, 10), (7, 5), (3, 10), (7, 15)]
    tails = {
        (11, 10): [(12, 10), (13, 10), (14, 10)],
        (3, 10): [(2, 10), (1, 10), (0, 10)],
        (7, 5): [(7, 4), (7, 3), (7, 2)],
        (7, 15): [(7, 16), (7, 17), (7, 18)],
    }
    # no ORDER at all: the anchored-region rule steers wavefronts onto the
    # stair tails dynamically; LaCAM-TAPF matching picks freely among all
    # physically-ready blocks
    all_tails = []
    for j in junctions:
        for t_i, cell in enumerate(tails[j]):
            T_BUILD[cell] = 3 - t_i
            T_FINAL[cell] = 0
        all_tails.extend(tails[j])

    # roof: gaps between pillars + hollow interior (side adhesion at z=5)
    interior = [(r, c) for r in range(5, 10) for c in range(7, 14)]
    DECK_CELLS = set(gaps) | set(interior)
    for c in DECK_CELLS:
        T_BUILD[c] = DECK_Z
        T_FINAL[c] = DECK_Z

    FORBIDDEN_CELLS = set(interior)
    COL_CELL = pillars[0]
    COL_H = 5
    RAMPS = [ring2] + [tails[j] for j in junctions]
    SCAFFOLD_CELLS = list(ring2) + list(all_tails)
    LINE_CELLS = set(ring2) | set(all_tails) | set(pillars)
    DEPOT = [(r, 22) for r in range(4, 12)]
    AGENT_STARTS = [(3 + i % 5, 20 - i // 5)
                    for i in range(int(os.environ.get("N_AGENTS", "20")))]


def configure_horse() -> None:
    """Trojan horse: 4 leg columns, a floating belly slab (z5..7, air below),
    a neck staircase (z8..9) and an overhanging 2x2 head (bottom z9, top z10).
    Built with a scaffold ring (h4, two stair tails) that is fully removed.
    The rump steps (T=5/6) and mane step (T=8) double as wavefront anchors.
    """
    global ROWS, COLS, COL_H, COL_CELL, RAMPS, SCAFFOLD_CELLS
    global T_BUILD, T_FINAL, LINE_CELLS, DECK_CELLS, DECK_LEVEL, ORDER
    global FORBIDDEN_CELLS, DEPOT, AGENT_STARTS
    ROWS, COLS = 16, 26
    T_BUILD, T_FINAL, ORDER = {}, {}, {}
    DECK_LEVEL = {}

    legs = [(5, 7), (8, 7), (5, 14), (8, 14)]
    body = [(r, c) for r in range(5, 9) for c in range(7, 15)]
    rump = [(6, 7), (7, 7)]            # tail slope: anchors for z6/z7 waves
    neck = [(6, 14), (7, 14)]          # mane step (T8) + neck top (T9)
    head = [(6, 15), (7, 15), (6, 16), (7, 16)]  # overhang, bottom z9

    for cell in body:
        if cell in legs:
            T_BUILD[cell] = 7          # leg column + body edge, solid
        else:
            DECK_LEVEL[cell] = 5       # floating belly bottom
            T_BUILD[cell] = 7
    T_BUILD[rump[0]] = 5
    T_BUILD[rump[1]] = 6
    T_BUILD[neck[0]] = 8
    T_BUILD[neck[1]] = 9
    for cell in head:
        DECK_LEVEL[cell] = 9
        T_BUILD[cell] = 10
    for cell, tgt in T_BUILD.items():
        T_FINAL[cell] = tgt

    # scaffold ring on three sides (front stays open for neck/head), h4,
    # fully removed afterwards; two perpendicular stair tails
    ring = ([(4, c) for c in range(6, 16)] + [(9, c) for c in range(6, 16)]
            + [(r, 6) for r in range(5, 9)])
    for cell in ring:
        T_BUILD[cell] = 4
        T_FINAL[cell] = 0
    tails = [[(10, 10), (11, 10), (12, 10)], [(3, 10), (2, 10), (1, 10)],
             [(4, 16), (4, 17), (4, 18)], [(9, 16), (9, 17), (9, 18)]]
    for tail in tails:
        for t_i, cell in enumerate(tail):
            T_BUILD[cell] = 3 - t_i
            T_FINAL[cell] = 0

    DECK_CELLS = set(DECK_LEVEL)
    FORBIDDEN_CELLS = set(DECK_LEVEL)  # no standing under floating slabs
    COL_CELL = legs[0]
    COL_H = 7
    RAMPS = [ring] + tails
    SCAFFOLD_CELLS = list(ring) + [c for tail in tails for c in tail]
    LINE_CELLS = set(ring) | {c for tail in tails for c in tail} | set(legs)
    DEPOT = [(r, 23) for r in range(4, 12)]
    AGENT_STARTS = [(3 + i % 5, 21 - i // 5)
                    for i in range(int(os.environ.get("N_AGENTS", "20")))]


def configure_goldengate() -> None:
    """Golden Gate v5: 18x52, towers h=16. Portal ring beams at z=16 plus
    viewer struts at z=9/11/13 give the real bridge's four stacked portal
    openings. Deck z=6; the catenary dips to z=7 mid-span so the mid deck is
    erected hanging from the cable (suspended erection). Scaffold work walls
    with passing pockets every 4 columns, stripped afterwards."""
    global ROWS, COLS, COL_H, COL_CELL, RAMPS, SCAFFOLD_CELLS
    global T_BUILD, T_FINAL, LINE_CELLS, DECK_CELLS, DECK_Z, DECK_LEVEL
    global ORDER, FORBIDDEN_CELLS, COLORS, COLORS3, SUSPENDERS, STRUTS
    global DEPOT, AGENT_STARTS
    ROWS, COLS = 18, 52
    T_BUILD, T_FINAL, ORDER = {}, {}, {}
    DECK_LEVEL, FORBIDDEN_CELLS, COLORS, COLORS3 = {}, set(), {}, {}
    SUSPENDERS = []
    orange = "#e0482a"
    cable_col = "#b03018"
    DECK_Z = 6
    TOWER_H = 16

    leg_cols = (16, 17, 34, 35)
    ring_cols = (16, 35)
    legs = [(r, c) for r in (7, 11) for c in leg_cols]
    for lg in legs:
        T_BUILD[lg] = TOWER_H
        T_FINAL[lg] = TOWER_H
        COLORS[lg] = orange
        COLORS3[(lg[0], lg[1], TOWER_H)] = cable_col

    # stepped 3-lane approaches (permanent)
    for r in (8, 9, 10):
        for k, c in enumerate((1, 2, 3, 4, 5, 6)):
            T_BUILD[(r, c)] = k + 1
            T_FINAL[(r, c)] = k + 1
            COLORS[(r, c)] = orange
        for k, c in enumerate((50, 49, 48, 47, 46, 45)):
            T_BUILD[(r, c)] = k + 1
            T_FINAL[(r, c)] = k + 1
            COLORS[(r, c)] = orange

    # suspended 3-lane deck at z=6, CONTINUOUS through the tower portals
    # (the roadway passes through the portal opening, like the real bridge)
    deck = [(r, c) for r in (8, 9, 10) for c in range(7, 45)]
    for cell in deck:
        T_BUILD[cell] = DECK_Z
        T_FINAL[cell] = DECK_Z
        DECK_LEVEL[cell] = DECK_Z
        COLORS[cell] = orange

    # the portal frame members are viewer-drawn (one float per cell cannot
    # hold both the through deck and a beam): three struts + a thick top
    # beam per tower face -> four stacked portal openings, visually joined
    # to both legs
    STRUTS.clear()
    for c in ring_cols:
        for z in (9, 11, 13):
            STRUTS.append({"c": c, "z": z, "r0": 7, "r1": 11})
        STRUTS.append({"c": c, "z": TOWER_H, "r0": 7, "r1": 11, "thick": 1})

    # cable profile: backstays climb 7..15, main span sags 15 -> 7 -> 15;
    # the low point sits one above the deck for suspended erection
    sag = {}
    for k, c in enumerate(range(7, 16)):
        sag[c] = 7 + k                      # west backstay 7..15
    main = [15, 14, 13, 12, 11, 10, 9, 8, 8, 9, 10, 11, 12, 13, 14, 15]
    for k, c in enumerate(range(18, 34)):
        sag[c] = main[k]
    for k, c in enumerate(range(36, 45)):
        sag[c] = 15 - k                     # east backstay 15..7
    cable_cells = []
    for r in (7, 11):
        for c, z in sag.items():
            cell = (r, c)
            cable_cells.append(cell)
            T_BUILD[cell] = z
            T_FINAL[cell] = z
            DECK_LEVEL[cell] = z
            COLORS[cell] = cable_col
            if z - 1 > DECK_Z:
                SUSPENDERS.append({"r": r, "c": c, "top": z, "bot": DECK_Z})
    DECK_CELLS = set(deck) | set(cable_cells)

    # research-informed scaffold: cables are hung DOWNHILL from the saddles
    # (workers walk the cable itself, each float hangs from the previous
    # one), so no work walls are needed. Towers still need one full-height
    # delivery stair per leg pair (every block trip must re-board the top),
    # which is ~60 scaffold blocks instead of ~700 for the old walls.
    wall_cells = []
    stair_rows = (6, 12) if os.environ.get("GG_DOUBLE_STAIRS", "0") == "0" \
        else (6, 12, 5, 13)
    for sr in stair_rows:
        for lg_c, dc in ((16, -1), (35, 1)):
            for j in range(TOWER_H - 1):
                cell = (sr, lg_c + dc * j)
                if cell in T_BUILD:
                    continue
                T_BUILD[cell] = TOWER_H - 1 - j
                wall_cells.append(cell)
    RAMPS = [wall_cells]
    SCAFFOLD_CELLS = list(wall_cells)
    LINE_CELLS = set(SCAFFOLD_CELLS) | set(legs)
    COL_CELL, COL_H = legs[0], TOWER_H

    n_gg = int(os.environ.get("GG_AGENTS", "16"))
    half = max(2, n_gg // 2)
    DEPOT = [(r, 0) for r in range(2, 2 + min(half, 14))] + \
        [(r, 51) for r in range(2, 2 + min(half, 14))]
    AGENT_STARTS = [(2 + i % 14, 1) for i in range(half)] + \
        [(2 + i % 14, 50) for i in range(n_gg - half)]


def configure_tommy() -> None:
    """Tommy Trojan: bronze warrior on a granite pedestal, RAISING the Sword
    of Knowledge skyward (ascending float chain, erected from a scaffold
    stair-wall behind him, stripped afterwards) and holding the Shield of
    Courage at his left. Colors: granite base, polished bronze body, gold
    belt/helmet, cardinal plume, steel blade."""
    global ROWS, COLS, COL_H, COL_CELL, RAMPS, SCAFFOLD_CELLS
    global T_BUILD, T_FINAL, LINE_CELLS, DECK_CELLS, DECK_Z, DECK_LEVEL
    global ORDER, FORBIDDEN_CELLS, COLORS, COLORS3, SUSPENDERS, STRUTS
    global DEPOT, AGENT_STARTS
    ROWS, COLS = 14, 20
    T_BUILD, T_FINAL, ORDER = {}, {}, {}
    DECK_LEVEL, DECK_CELLS, FORBIDDEN_CELLS = {}, set(), set()
    COLORS, COLORS3 = {}, {}
    SUSPENDERS, STRUTS = [], []
    granite = "#6f7076"
    bronze = "#8a6420"
    bronze_lt = "#a67c33"
    bronze_dk = "#5f451a"
    gold = "#ffc72c"
    cardinal = "#9d2235"
    steel = "#d9dee8"
    shadow = "#3d3320"

    # granite pedestal, rows 6..11 x cols 6..12, h=2
    for r in range(6, 12):
        for c in range(6, 13):
            T_BUILD[(r, c)] = 2
            T_FINAL[(r, c)] = 2
            COLORS[(r, c)] = granite

    # body: 3x2 column blob (cols 8..10 = legs/torso, rows 8..9), h=12
    body_cols = (8, 9, 10)
    for r in (8, 9):
        for c in body_cols:
            T_BUILD[(r, c)] = 12
            T_FINAL[(r, c)] = 12
            COLORS[(r, c)] = bronze
            COLORS3[(r, c, 1)] = granite
            COLORS3[(r, c, 2)] = granite
            for z in (3, 4, 5, 6, 7):     # legs: center column reads as gap
                COLORS3[(r, c, z)] = shadow if c == 9 else bronze
            COLORS3[(r, c, 8)] = bronze_dk   # pleated skirt hem
            COLORS3[(r, c, 9)] = gold        # championship belt
    # head + helmet on the center front column; plume rides the back column
    T_BUILD[(8, 9)] = 13
    T_FINAL[(8, 9)] = 13
    COLORS3[(8, 9, 12)] = bronze_lt          # face
    COLORS3[(8, 9, 13)] = gold               # helmet
    T_BUILD[(9, 9)] = 13
    T_FINAL[(9, 9)] = 13
    COLORS3[(9, 9, 13)] = cardinal           # mohawk plume
    # shoulder caps
    COLORS3[(8, 8, 12)] = bronze_dk
    COLORS3[(8, 10, 12)] = bronze_dk

    # Shield of Courage: tall oval wall on the pedestal at his left
    T_BUILD[(8, 6)] = 9
    T_FINAL[(8, 6)] = 9
    COLORS[(8, 6)] = bronze_dk
    COLORS3[(8, 6, 6)] = gold                # shield boss
    T_BUILD[(9, 6)] = 8
    T_FINAL[(9, 6)] = 8
    COLORS[(9, 6)] = bronze_dk
    # left forearm reaching the shield rim (single float)
    T_BUILD[(8, 7)] = 8
    T_FINAL[(8, 7)] = 8
    DECK_LEVEL[(8, 7)] = 8
    COLORS[(8, 7)] = bronze

    # Sword of Knowledge: raised arm + blade as an ascending float chain
    sword = {(8, 11): 12, (8, 12): 13, (8, 13): 14, (8, 14): 15}
    for cell, z in sword.items():
        T_BUILD[cell] = z
        T_FINAL[cell] = z
        DECK_LEVEL[cell] = z
        COLORS[cell] = bronze if cell[1] <= 12 else steel
    COLORS3[(8, 12, 13)] = gold              # crossguard
    DECK_CELLS = {(8, 7)} | set(sword)

    # scaffold stair-wall behind the statue (row 7): ground tail c0..c5,
    # over the pedestal c6..c12 (T_FINAL=2 keeps the pedestal), then rising
    # beside the sword to c14; gives stands for torso, head and every float
    wall_cells = []
    for c in range(0, 15):
        z = c + 1
        cell = (7, c)
        T_BUILD[cell] = max(z, T_BUILD.get(cell, 0))
        if cell not in T_FINAL:
            T_FINAL[cell] = 0
        wall_cells.append(cell)
    RAMPS = [wall_cells]
    SCAFFOLD_CELLS = list(wall_cells)
    LINE_CELLS = set(SCAFFOLD_CELLS)
    COL_CELL, COL_H = (8, 9), 13

    DEPOT = [(r, 18) for r in range(3, 11)]
    AGENT_STARTS = [(3 + i, 16) for i in range(8)]


def configure_exp() -> None:
    """Parametric scaffold-research scene, driven by env vars:
      EXP_F      tower footprint FxF          (default 2)
      EXP_H      tower height                 (default 10)
      EXP_K      number of stairs 1..4        (default 1)
      EXP_N      number of agents             (default 8)
      EXP_STYLE  stair style: straight | hug  (default straight)
      EXP_SIDES  sides for stairs: spread | same (default spread)
    Tower at the map center, one straight ramp per stair descending away
    (or hugging the tower for style=hug). Depot on the east edge."""
    global ROWS, COLS, COL_H, COL_CELL, RAMPS, SCAFFOLD_CELLS
    global T_BUILD, T_FINAL, LINE_CELLS, DECK_CELLS, DECK_Z, DECK_LEVEL
    global ORDER, FORBIDDEN_CELLS, COLORS, COLORS3, SUSPENDERS, STRUTS
    global DEPOT, AGENT_STARTS
    F = int(os.environ.get("EXP_F", "2"))
    H = int(os.environ.get("EXP_H", "10"))
    K = int(os.environ.get("EXP_K", "1"))
    N = int(os.environ.get("EXP_N", "8"))
    style = os.environ.get("EXP_STYLE", "straight")
    sides_mode = os.environ.get("EXP_SIDES", "spread")
    margin = H + 3
    ROWS = COLS = 2 * margin + F
    T_BUILD, T_FINAL, ORDER = {}, {}, {}
    DECK_LEVEL, DECK_CELLS, FORBIDDEN_CELLS = {}, set(), set()
    COLORS, COLORS3, SUSPENDERS, STRUTS = {}, {}, [], []
    shape = os.environ.get("EXP_SHAPE", "tower")
    W = int(os.environ.get("EXP_W", "12"))
    r0 = c0 = margin
    if shape == "wall":
        # 1-wide wall of length W at height H, running east-west
        COLS = max(COLS, W + 2 * margin)
        for k in range(W):
            T_BUILD[(r0, c0 + k)] = H
            T_FINAL[(r0, c0 + k)] = H
            COLORS[(r0, c0 + k)] = "#e8b04a"
    else:
        for r in range(r0, r0 + F):
            for c in range(c0, c0 + F):
                T_BUILD[(r, c)] = H
                T_FINAL[(r, c)] = H
                COLORS[(r, c)] = "#e8b04a"
    # stairs: attach to the middle of each chosen side
    if shape == "wall":
        # K stairs evenly spaced along the SOUTH face of the wall,
        # descending southward
        ramps = []
        for k in range(K):
            bc = c0 + (W * (2 * k + 1)) // (2 * K)
            cells = [(r0 + 1 + j, bc) for j in range(H - 1)]
            for j, cell in enumerate(cells):
                T_BUILD[cell] = H - 1 - j
            ramps.append(cells)
        RAMPS = ramps
        SCAFFOLD_CELLS = [c for rp in ramps for c in rp]
        LINE_CELLS = set(SCAFFOLD_CELLS) | {(r0, c0 + k) for k in range(W)}
        COL_CELL, COL_H = (r0, c0), H
        DEPOT = [(3 + i, COLS - 1) for i in range(max(N, 4))]
        AGENT_STARTS = [(3 + i, COLS - 3) for i in range(N)]
        return
    mid = F // 2
    sides = [
        ((r0 - 1, c0 + mid), (-1, 0), (0, 1)),   # north, hug dir east
        ((r0 + F, c0 + mid), (1, 0), (0, 1)),    # south
        ((r0 + mid, c0 - 1), (0, -1), (1, 0)),   # west
        ((r0 + mid, c0 + F), (0, 1), (1, 0)),    # east
    ]
    if sides_mode == "same":
        base, (dr, dc), (hr, hc) = sides[0]
        chosen = []
        for k in range(K):
            br, bc = base[0], base[1] + 2 * k  # parallel stairs, same side
            chosen.append(((br, bc), (dr, dc), (hr, hc)))
    else:
        chosen = sides[:K]
    ramps = []
    for (br, bc), (dr, dc), (hr, hc) in chosen:
        cells = []
        if style == "hug":
            # first cell beside the tower, then wrap around it clockwise-ish
            cur = (br, bc)
            vec = (hr, hc)
            for k in range(H - 1):
                cells.append(cur)
                cur = (cur[0] + vec[0], cur[1] + vec[1])
        else:
            for k in range(H - 1):
                cells.append((br + dr * k, bc + dc * k))
        for k, cell in enumerate(cells):
            T_BUILD[cell] = H - 1 - k
        ramps.append(cells)
    RAMPS = ramps
    SCAFFOLD_CELLS = [c for rp in ramps for c in rp]
    LINE_CELLS = set(SCAFFOLD_CELLS) | set(
        (r, c) for r in range(r0, r0 + F) for c in range(c0, c0 + F))
    COL_CELL, COL_H = (r0, c0), H
    DEPOT = [(3 + i, COLS - 1) for i in range(max(N, 4))]
    AGENT_STARTS = [(3 + i, COLS - 3) for i in range(N)]


def configure_uscgate() -> None:
    """USC-style gate: two cardinal brick pillars with a gold lintel slab at
    z=5 (side adhesion from the pillar tops), low side walls, and a gold
    "USC" ground inlay in front of the gate."""
    global ROWS, COLS, COL_H, COL_CELL, RAMPS, SCAFFOLD_CELLS
    global T_BUILD, T_FINAL, LINE_CELLS, DECK_CELLS, DECK_Z, DECK_LEVEL
    global ORDER, FORBIDDEN_CELLS, COLORS, COLORS3, DEPOT, AGENT_STARTS
    ROWS, COLS = 16, 26
    T_BUILD, T_FINAL, ORDER = {}, {}, {}
    DECK_LEVEL, FORBIDDEN_CELLS, COLORS, COLORS3 = {}, set(), {}, {}
    cardinal, gold = "#9d2235", "#ffc72c"
    p_a, p_b = (5, 7), (5, 15)
    DECK_Z = 5
    for p in (p_a, p_b):
        T_BUILD[p] = 5
        T_FINAL[p] = 5
        COLORS[p] = cardinal
        COLORS3[(p[0], p[1], 5)] = gold  # gold pillar cap
    lintel = [(5, c) for c in range(8, 15)]
    for c in lintel:
        T_BUILD[c] = DECK_Z
        T_FINAL[c] = DECK_Z
        COLORS[c] = gold
    DECK_CELLS = set(lintel)
    walls = [(5, c) for c in range(3, 7)] + [(5, c) for c in range(16, 20)]
    for w in walls:
        T_BUILD[w] = 1
        T_FINAL[w] = 1
        COLORS[w] = cardinal
    # "USC" ground inlay (3x5 font), gold
    font = {
        "U": ["X.X", "X.X", "X.X", "X.X", "XXX"],
        "S": ["XXX", "X..", "XXX", "..X", "XXX"],
        "C": ["XXX", "X..", "X..", "X..", "XXX"],
    }
    letters = []
    for li, ch in enumerate("USC"):
        c0 = 8 + li * 4
        for rr, rowtxt in enumerate(font[ch]):
            for cc, mark in enumerate(rowtxt):
                if mark == "X":
                    letters.append((8 + rr, c0 + cc))
    for cell in letters:
        T_BUILD[cell] = 1
        T_FINAL[cell] = 1
        COLORS[cell] = gold
    # scaffold stairs north of each pillar
    ramps = []
    for p in (p_a, p_b):
        ramp = [(p[0] - 1 - k, p[1]) for k in range(4)]  # targets 4..1
        for k, cell in enumerate(ramp):
            T_BUILD[cell] = 4 - k
        ramps.append(ramp)
    RAMPS = ramps
    SCAFFOLD_CELLS = [c for ramp in ramps for c in ramp]
    LINE_CELLS = set(SCAFFOLD_CELLS) | {p_a, p_b} | set(walls) | set(letters)
    COL_CELL, COL_H = p_a, 5
    DEPOT = [(r, 23) for r in range(3, 11)]
    AGENT_STARTS = [(3 + i, 21) for i in range(8)]


def configure_symbot() -> None:
    """SymBot v2: Symbotic's bots are low-slung green case-carrying vehicles.
    Solid green body (h2, 3x10), black wheel pods at the four corners (h1,
    doubling as permanent access steps), dark sensor nose, orange case
    payload (h3) amidships. No scaffold needed."""
    global ROWS, COLS, COL_H, COL_CELL, RAMPS, SCAFFOLD_CELLS
    global T_BUILD, T_FINAL, LINE_CELLS, DECK_CELLS, DECK_Z, DECK_LEVEL
    global ORDER, FORBIDDEN_CELLS, COLORS, DEPOT, AGENT_STARTS
    ROWS, COLS = 14, 24
    T_BUILD, T_FINAL, ORDER = {}, {}, {}
    DECK_LEVEL, DECK_CELLS, FORBIDDEN_CELLS, COLORS = {}, set(), set(), {}
    green, black, dark, case = "#00b140", "#16161a", "#2d3436", "#ff8c00"
    body_rows, body_cols = (6, 7, 8), range(5, 15)
    wheels = [(6, 5), (8, 5), (6, 14), (8, 14)]
    nose = (7, 15)  # sensor nose at the front
    case_cells = [(7, 9), (7, 10)]  # carried case amidships, one above deck
    for r in body_rows:
        for c in body_cols:
            cell = (r, c)
            if cell in wheels:
                T_BUILD[cell] = 1
                T_FINAL[cell] = 1
                COLORS[cell] = black
            else:
                T_BUILD[cell] = 2
                T_FINAL[cell] = 2
                COLORS[cell] = green
    T_BUILD[nose] = 1
    T_FINAL[nose] = 1
    COLORS[nose] = dark
    for cell in case_cells:
        T_BUILD[cell] = 3
        T_FINAL[cell] = 3
        COLORS[cell] = case
    RAMPS = []
    SCAFFOLD_CELLS = []
    LINE_CELLS = set(wheels) | {nose}
    COL_CELL, COL_H = (7, 9), 3
    DEPOT = [(r, 21) for r in range(3, 11)]
    AGENT_STARTS = [(3 + i, 19) for i in range(6)]


def _add_pyramid(t_build: dict, r0: int, c0: int, base: int, levels: int) -> None:
    for lvl in range(1, levels + 1):
        inset = lvl - 1
        side = base - 2 * inset
        if side <= 0:
            break
        for r in range(r0 + inset, r0 + inset + side):
            for c in range(c0 + inset, c0 + inset + side):
                t_build[(r, c)] = lvl


def configure_pyramid() -> None:
    """Plain pyramid via the unified planner (integrated assignment)."""
    global ROWS, COLS, COL_H, COL_CELL, RAMPS, SCAFFOLD_CELLS
    global T_BUILD, T_FINAL, LINE_CELLS, DECK_CELLS, DECK_LEVEL, ORDER
    global FORBIDDEN_CELLS, DEPOT, AGENT_STARTS
    ROWS, COLS = 14, 22
    T_BUILD, T_FINAL, ORDER = {}, {}, {}
    DECK_LEVEL = {}
    _add_pyramid(T_BUILD, 3, 4, 7, 4)
    T_FINAL = dict(T_BUILD)
    DECK_CELLS = set()
    FORBIDDEN_CELLS = set()
    RAMPS = []
    SCAFFOLD_CELLS = []
    LINE_CELLS = set()
    COL_CELL = (6, 7)
    COL_H = 4
    DEPOT = [(r, 20) for r in range(3, 11)]
    AGENT_STARTS = [(3 + i, 18) for i in range(6)]


def configure_scene() -> None:
    """Giza scene: large pyramid + small pyramid (self-scaffolding) plus an
    obelisk with a temporary staircase (built then dismantled)."""
    global ROWS, COLS, COL_H, COL_CELL, RAMPS, SCAFFOLD_CELLS
    global T_BUILD, T_FINAL, LINE_CELLS, DEPOT, AGENT_STARTS
    ROWS, COLS = 16, 26
    T_BUILD = {}
    _add_pyramid(T_BUILD, 4, 2, 7, 4)     # large pyramid, rows 4-10, cols 2-8
    _add_pyramid(T_BUILD, 10, 11, 5, 3)   # small pyramid, rows 10-14, cols 11-15
    COL_CELL = (4, 13)
    COL_H = 6
    ramp = [(4, 13 + 1 + k) for k in range(COL_H - 1)]  # east of obelisk
    T_FINAL = dict(T_BUILD)
    T_FINAL[COL_CELL] = COL_H
    T_BUILD[COL_CELL] = COL_H
    for k, cell in enumerate(ramp):
        T_BUILD[cell] = COL_H - 1 - k
    RAMPS = [ramp]
    SCAFFOLD_CELLS = list(ramp)
    LINE_CELLS = set(ramp) | {COL_CELL}
    DEPOT = [(r, 24) for r in range(3, 13)]
    AGENT_STARTS = [(3 + i % 10, 22) for i in range(10)]


@dataclass
class Task:
    cell: Coord
    level: int  # place: resulting height; remove: height of removed block
    kind: str   # "place" | "remove"


@dataclass
class Agent:
    name: str
    pos: Coord
    carrying: bool = False
    cargo: Optional[str] = None  # "new" | "scrap"
    task: Optional[Task] = None  # recovered from solver assignment each round
    goal: Optional[Coord] = None
    intent: str = "hold"  # depot | stand | return | hold | action
    action: Optional[str] = None  # pick | place | remove | deposit
    action_fails: int = 0  # consecutive blocked completions; give up past cap
    action_age: int = 0  # rounds this action has been pending
    prev_goal: Optional[Coord] = None  # last round's assigned goal (stickiness)
    path_hist: List[Coord] = field(default_factory=list)
    carry_hist: List[int] = field(default_factory=list)


def dijkstra(src: Coord, heights: List[List[int]]) -> List[List[int]]:
    import heapq
    dist = [[INF] * COLS for _ in range(ROWS)]
    dist[src[0]][src[1]] = 0
    pq = [(0, src)]
    while pq:
        d, (r, c) = heapq.heappop(pq)
        if d > dist[r][c]:
            continue
        for dr, dc in NBRS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < ROWS and 0 <= nc < COLS):
                continue
            dh = abs(heights[nr][nc] - heights[r][c])
            if dh > 1:
                continue
            nd = d + (CLIMB_COST if dh == 1 else 1)
            if nd < dist[nr][nc]:
                dist[nr][nc] = nd
                heapq.heappush(pq, (nd, (nr, nc)))
    return dist


def component_labels(heights: List[List[int]]) -> List[List[int]]:
    """Connected-component labels under the |dh| <= 1 move rule (edges are
    symmetric, so reachability checks reduce to label equality)."""
    comp = [[-1] * COLS for _ in range(ROWS)]
    label = 0
    for r0 in range(ROWS):
        for c0 in range(COLS):
            if comp[r0][c0] != -1:
                continue
            comp[r0][c0] = label
            stack = [(r0, c0)]
            while stack:
                r, c = stack.pop()
                for dr, dc in NBRS:
                    nr, nc = r + dr, c + dc
                    if (0 <= nr < ROWS and 0 <= nc < COLS and comp[nr][nc] == -1
                            and abs(heights[nr][nc] - heights[r][c]) <= 1):
                        comp[nr][nc] = label
                        stack.append((nr, nc))
            label += 1
    return comp


def write_map(path: Path) -> None:
    lines = ["type octile", f"height {ROWS}", f"width {COLS}", "map"]
    lines += ["." * COLS for _ in range(ROWS)]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def expand_schedule(entries: Sequence[Dict[str, int]], fallback: Coord, horizon: int) -> List[Coord]:
    path = [fallback] * (horizon + 1)
    cur = fallback
    nodes = sorted(((e["t"], (e["x"], e["y"])) for e in entries), key=lambda x: x[0])
    idx = 0
    for t in range(horizon + 1):
        while idx < len(nodes) and nodes[idx][0] == t:
            cur = nodes[idx][1]
            idx += 1
        path[t] = cur
    return path


def main() -> int:
    global DECK_LEVEL
    if DECK_CELLS and not DECK_LEVEL:
        DECK_LEVEL = {c: DECK_Z for c in DECK_CELLS}
    WORK.mkdir(parents=True, exist_ok=True)
    heights = [[0] * COLS for _ in range(ROWS)]

    def deck_task(cell: Coord, level: int) -> bool:
        return cell in DECK_LEVEL and level == DECK_LEVEL[cell]

    def add_auto_pockets() -> None:
        """Generic parallelism transform: every long 1-wide scaffold corridor
        gets passing-pocket cells (one level below the host, on any free side
        cell) every AUTO_POCKET_EVERY cells along the corridor. Pockets are
        scaffold themselves (stripped afterwards) and turn the corridor
        locally 2-wide so several workers can slip past each other."""
        scaffold0 = [c for c, t in T_BUILD.items() if t > T_FINAL.get(c, 0)]
        sset = set(scaffold0)
        blocked = set(T_BUILD) | set(DEPOT) | set(AGENT_STARTS) | FORBIDDEN_CELLS
        seen: set = set()
        added = 0
        every = (AUTO_POCKET_EVERY if AUTO_POCKET_EVERY_SET
                 else (POCKET_EVERY_HINT or AUTO_POCKET_EVERY))
        for cell in scaffold0:
            if cell in seen:
                continue
            comp = [cell]
            seen.add(cell)
            qi = 0
            while qi < len(comp):
                cur = comp[qi]
                qi += 1
                for dr, dc in NBRS:
                    nb = (cur[0] + dr, cur[1] + dc)
                    if nb in sset and nb not in seen:
                        seen.add(nb)
                        comp.append(nb)
            if len(comp) < SCAFFOLD_MIN:
                continue  # short stairs stay as-is
            # 2-core pruning: cells on cycles have two ways around and need
            # no pockets; only tree-like corridor cells (spurs, open walls)
            # can deadlock face-to-face
            deg = {x: sum(1 for dr, dc in NBRS if (x[0] + dr, x[1] + dc) in sset)
                   for x in comp}
            core_deg = dict(deg)
            peel = [x for x in comp if core_deg[x] <= 1]
            core = set(comp)
            while peel:
                x = peel.pop()
                if x not in core:
                    continue
                core.discard(x)
                for dr, dc in NBRS:
                    nb = (x[0] + dr, x[1] + dc)
                    if nb in core:
                        core_deg[nb] -= 1
                        if core_deg[nb] <= 1:
                            peel.append(nb)
            start = min(comp, key=lambda x: (deg[x], x))
            order = [start]
            seen_c = {start}
            qi = 0
            while qi < len(order):
                cur = order[qi]
                qi += 1
                for dr, dc in NBRS:
                    nb = (cur[0] + dr, cur[1] + dc)
                    if nb in sset and nb not in seen_c:
                        seen_c.add(nb)
                        order.append(nb)
            for idx, cur in enumerate(order):
                if idx % every != every // 2:
                    continue
                if cur in core:
                    continue  # on a cycle: two ways around, no pocket needed
                z = T_BUILD[cur] - (0 if AUTO_POCKET_MODE == "flat" else 1)
                if z < 1:
                    continue
                for dr, dc in NBRS:
                    nb = (cur[0] + dr, cur[1] + dc)
                    if not (0 <= nb[0] < ROWS and 0 <= nb[1] < COLS):
                        continue
                    if nb in blocked:
                        continue
                    T_BUILD[nb] = z          # T_FINAL absent -> stripped
                    SCAFFOLD_CELLS.append(nb)
                    LINE_CELLS.add(nb)
                    POCKET_CELLS.add(nb)
                    blocked.add(nb)
                    added += 1
                    break
        if added:
            print(f"auto-pockets: added {added} passing cells", flush=True)

    # adaptive corridor policy (research result): HEAVY-TRAFFIC corridors
    # (long single-file scaffold AND >=12 agents) get dense same-level
    # landings (flat, every 2) plus strong stickiness 12 - the landings make
    # strong stickiness safe, and together they cut shuffling (rev% 17->9 on
    # goldengate). Light scenes get sparse step pockets (every 8) and mild
    # stickiness 6 (dense pockets and strong stickiness both HURT there).
    global STICKY_BONUS, AUTO_POCKET_MODE, AUTO_POCKET_EVERY
    scaffold_all = [c for c, t in T_BUILD.items() if t > T_FINAL.get(c, 0)]
    sset_ = set(scaffold_all)
    long_corridor = False
    seen_ = set()
    for cell in scaffold_all:
        if cell in seen_:
            continue
        comp_ = [cell]
        seen_.add(cell)
        qi_ = 0
        while qi_ < len(comp_):
            cur_ = comp_[qi_]
            qi_ += 1
            for dr, dc in NBRS:
                nb_ = (cur_[0] + dr, cur_[1] + dc)
                if nb_ in sset_ and nb_ not in seen_:
                    seen_.add(nb_)
                    comp_.append(nb_)
        if len(comp_) >= SCAFFOLD_MIN:
            long_corridor = True
            break
    heavy = long_corridor and len(AGENT_STARTS) >= 12
    if STICKY_BONUS == 0:
        STICKY_BONUS = 12 if heavy else 6
    if not AUTO_POCKET_MODE:
        AUTO_POCKET_MODE = "flat" if heavy else "step"
    if AUTO_POCKET_EVERY == 0:
        AUTO_POCKET_EVERY = 2 if heavy else 8

    if AUTO_POCKETS:
        add_auto_pockets()

    build_pending: List[Task] = []
    for cell, tgt in T_BUILD.items():
        if cell in DECK_LEVEL:
            dz = DECK_LEVEL[cell]
            build_pending.append(Task(cell=cell, level=dz, kind="place"))
            for l in range(dz + 1, tgt + 1):  # stacked layers above the slab
                build_pending.append(Task(cell=cell, level=l, kind="place"))
        else:
            for l in range(1, tgt + 1):
                build_pending.append(Task(cell=cell, level=l, kind="place"))
    total_places = len(build_pending)
    demolition: List[Task] = [Task(cell=c, level=l, kind="remove")
                              for c in SCAFFOLD_CELLS
                              for l in range(T_FINAL.get(c, 0) + 1, T_BUILD[c] + 1)]
    demolition_start_t = 0
    peaks = [[0] * COLS for _ in range(ROWS)]

    agents = [Agent(name=f"a{i}", pos=p, path_hist=[p], carry_hist=[0]) for i, p in enumerate(AGENT_STARTS)]
    terrain_events: List[Dict[str, int]] = []
    pick_events: List[Dict[str, int]] = []
    deposit_events: List[Dict[str, int]] = []
    t_global = 0
    round_no = 0
    stall_rounds = 0
    remaining_places = total_places
    evict_names: set = set()
    progress_stall = 0
    task_cooldown: Dict[Tuple, int] = {}
    throttle_until = [0]
    dist_cache: Dict[Coord, List[List[int]]] = {}
    comp_cache: List[List[List[int]]] = []
    dist_cache_ver = [-1]

    scaffold_cells_set = set(SCAFFOLD_CELLS)

    def ready_place(task: Task) -> bool:
        r, c = task.cell
        if deck_task(task.cell, task.level):
            if heights[r][c] >= task.level:
                return False
            # side adhesion: a face-adjacent SOLID at this z (top >= z and the
            # neighbour's column is not hollow at this z)
            def solid_at(n: Coord) -> bool:
                if heights[n[0]][n[1]] < task.level:
                    return False
                return DECK_LEVEL.get(n, 0) <= task.level

            def hung_from(n: Coord) -> bool:
                # suspended erection: a floating block exactly one level up
                # (e.g. the main cable) can carry this slab from above-side
                return (DECK_LEVEL.get(n) == task.level + 1
                        and heights[n[0]][n[1]] == task.level + 1)
            return any(0 <= r + dr < ROWS and 0 <= c + dc < COLS
                       and (solid_at((r + dr, c + dc))
                            or hung_from((r + dr, c + dc)))
                       for dr, dc in NBRS)
        if heights[r][c] != task.level - 1:
            return False
        # locomotive rule: never rise more than 1 above an unfinished
        # neighbour that still needs to grow to this level or beyond
        # (side-placement makes any 1-step ledge buildable, so keeping the
        # height field 1-Lipschitz over unfinished cells guarantees progress)
        # NOTE >= task.level - 1: a neighbour topping out exactly one below
        # us can only ever be built from OUR column, so we must not outgrow
        # it either (or its last level becomes unreachable forever)
        for dr, dc in NBRS:
            n = (r + dr, c + dc)
            if not (0 <= n[0] < ROWS and 0 <= n[1] < COLS) or n not in T_BUILD:
                continue
            hn = heights[n[0]][n[1]]
            if n in DECK_LEVEL and hn < DECK_LEVEL[n]:
                hn = DECK_LEVEL[n] - 1  # unroofed slab counts as one-below-slab
            if hn < T_BUILD[n] and T_BUILD[n] >= task.level - 1:
                if task.level > hn + 1:
                    return False
        if task.level == 1 and task.cell in LINE_CELLS and task.cell not in ORDER:
            return True  # 1-wide line: flat ground alongside provides a stand
        def _order(cell):
            return ORDER.get(cell, T_BUILD.get(cell, 0))
        for dr, dc in NBRS:
            n = (r + dr, c + dc)
            if n in DECK_LEVEL and heights[n[0]][n[1]] < DECK_LEVEL[n]:
                continue  # unroofed floating cells exert no bottom-up ordering
            if n in T_BUILD and _order(n) > _order(task.cell):
                if heights[n[0]][n[1]] < min(task.level, T_BUILD[n]):
                    return False
        return True

    def anchored_after(xc: Coord, l: int) -> bool:
        """After raising xc to l, every unfinished-at-l region among its
        neighbours must still touch an anchor: a cell permanently sitting at
        height l-1 (finished cell with T_BUILD == l-1, or plain ground for
        l == 1). This lets LaCAM-TAPF choose freely among ready blocks while
        making wavefronts converge onto stair tails automatically."""
        def in_g(v: Coord) -> bool:
            if v in DECK_LEVEL and heights[v[0]][v[1]] < DECK_LEVEL[v]:
                return False  # unroofed floating cell: not part of any region
            return (v in T_BUILD and v != xc
                    and T_BUILD[v] >= l and heights[v[0]][v[1]] < l)

        def is_anchor(a: Coord) -> bool:
            # a cell whose FINAL height is exactly l-1 and which has not yet
            # grown past it: it either already provides the stand or can wait
            # at l-1 until the region is finished
            if not (0 <= a[0] < ROWS and 0 <= a[1] < COLS):
                return False
            if a in DECK_LEVEL and heights[a[0]][a[1]] < DECK_LEVEL[a]:
                return False  # not yet roofed: cannot promise a stand
            if heights[a[0]][a[1]] > l - 1:
                return False
            return T_BUILD.get(a, 0) == l - 1  # ground (0) anchors l == 1

        seen: set = set()
        for dr, dc in NBRS:
            y = (xc[0] + dr, xc[1] + dc)
            if not (0 <= y[0] < ROWS and 0 <= y[1] < COLS) or y in seen or not in_g(y):
                continue
            comp = [y]
            seen.add(y)
            ok = False
            qi = 0
            while qi < len(comp):  # expand fully: seen is shared across regions
                v = comp[qi]
                qi += 1
                for dr2, dc2 in NBRS:
                    w = (v[0] + dr2, v[1] + dc2)
                    if not (0 <= w[0] < ROWS and 0 <= w[1] < COLS):
                        continue
                    if is_anchor(w):
                        ok = True
                    elif w not in seen and in_g(w):
                        seen.add(w)
                        comp.append(w)
            if not ok:
                return False
        return True

    def ready_remove(task: Task) -> bool:
        """Removable iff (a) it's the top block, (b) every deeper placement
        that may need it as a stand has already been built (peak heights),
        and (c) it is not the last remaining stand of any scaffold neighbor
        currently exactly one above it, and (d) removing it must not cut any
        remaining un-stripped scaffold off from the ground (no orphan
        islands: workers must always retain a way up)."""
        r, c = task.cell
        h = task.level
        if heights[r][c] != h:
            return False
        if peaks[r][c] < T_BUILD[task.cell]:
            return False  # this scaffold cell hasn't finished growing yet
        for dr, dc in NBRS:
            y = (r + dr, c + dc)
            if not (0 <= y[0] < ROWS and 0 <= y[1] < COLS):
                continue
            if y in T_BUILD and T_BUILD[y] > T_BUILD[task.cell]:
                # deeper neighbor may need to stand on x at h to build (y, h+1)
                if peaks[y[0]][y[1]] < min(h + 1, T_BUILD[y]):
                    return False
            if not (y in T_BUILD and T_BUILD[y] > T_FINAL.get(y, 0)):
                continue  # not scaffold
            if heights[y[0]][y[1]] <= T_FINAL.get(y, 0):
                continue  # already stripped to final: needs no stand anymore
            # strip locomotive rule: never sink more than one level below a
            # still-taller unfinished scaffold neighbour (keeps descent
            # ladders intact while letting strip waves pipeline)
            if (heights[y[0]][y[1]] > T_FINAL.get(y, 0)
                    and heights[y[0]][y[1]] > h):
                return False
            if heights[y[0]][y[1]] != h + 1:
                continue
            alt = False
            for dr2, dc2 in NBRS:
                z = (y[0] + dr2, y[1] + dc2)
                if z == (r, c) or not (0 <= z[0] < ROWS and 0 <= z[1] < COLS):
                    continue
                if heights[z[0]][z[1]] == h:
                    alt = True
                    break
            if not alt:
                return False
        # (d) no orphan islands: simulate the removal and verify that every
        # remaining above-final scaffold block stays reachable from ground
        # level (|dh| <= 1 walk).  local flood fill from the changed cell's
        # region only (cheap: scaffold sets are small).
        rest = [y for y in scaffold_cells_set
                if heights[y[0]][y[1]] > T_FINAL.get(y, 0) and y != task.cell]
        if rest:
            heights[r][c] -= 1  # hypothetical removal
            try:
                ok = True
                for y in rest:
                    # walk from y: can we still reach any ground-level cell?
                    seen_l = {y}
                    stack = [y]
                    found = False
                    while stack:
                        cur = stack.pop()
                        ch = heights[cur[0]][cur[1]]
                        if ch == 0:
                            found = True
                            break
                        for dr, dc in NBRS:
                            n = (cur[0] + dr, cur[1] + dc)
                            if (0 <= n[0] < ROWS and 0 <= n[1] < COLS
                                    and n not in seen_l
                                    and abs(heights[n[0]][n[1]] - ch) <= 1):
                                seen_l.add(n)
                                stack.append(n)
                    if not found:
                        ok = False
                        break
            finally:
                heights[r][c] += 1
            if not ok:
                return False
        return True

    def task_stand_cells(task: Task) -> List[Coord]:
        if task.kind == "place" and deck_task(task.cell, task.level):
            # floating slab: lay from its own level or one above (stepping
            # half a level down to bridge sideways is a legal reach)
            want = (task.level, task.level + 1)
        elif task.kind == "place":
            want = (task.level - 1, task.level)  # from below or from the side
        else:
            want = (task.level - 1, task.level)  # remove: reach up or sideways
        out = []
        for dr, dc in NBRS:
            n = (task.cell[0] + dr, task.cell[1] + dc)
            if (0 <= n[0] < ROWS and 0 <= n[1] < COLS
                    and not (n in FORBIDDEN_CELLS and heights[n[0]][n[1]] < DECK_LEVEL.get(n, DECK_Z))
                    and heights[n[0]][n[1]] in want):
                out.append(n)
        return out

    def work_cells() -> set:
        out = set()
        for tsk in build_pending:
            out.add(tsk.cell)
        for tsk in demolition:
            out.add(tsk.cell)
        for r in range(ROWS):
            for c in range(COLS):
                if heights[r][c] > 0:
                    out.add((r, c))
        out |= {c for c in FORBIDDEN_CELLS if heights[c[0]][c[1]] < DECK_LEVEL.get(c, DECK_Z)}
        return out

    removal_latch = not DECK_CELLS  # bridge scene: scaffold locked until latch opens
    import time as _time
    _wall_start = _time.time()
    while round_no < MAX_ROUNDS and t_global < MAX_STEPS:
        if MAX_WALL_SEC and _time.time() - _wall_start > MAX_WALL_SEC:
            done_actions = (total_places - remaining_places) + \
                (len(terrain_events) - (total_places - remaining_places))
            acts = len(terrain_events) + len(pick_events) + len(deposit_events)
            print(f"BUDGET t={t_global} rounds={round_no} "
                  f"placed={total_places - remaining_places}/{total_places} "
                  f"demo_left={len(demolition)} actions={acts} "
                  f"thr={acts / max(t_global, 1):.3f}", flush=True)
            return 0
        round_no += 1
        if not removal_latch and remaining_places == 0 and all(
                heights[a.pos[0]][a.pos[1]] == 0 for a in agents):
            removal_latch = True

        if (remaining_places == 0 and not demolition
                and not any(a.carrying or a.action for a in agents)):
            break

        if stall_rounds >= 50:
            raise RuntimeError(f"stalled at round {round_no}")
        if progress_stall >= 800:
            raise RuntimeError(
                f"no completed action for {progress_stall} rounds "
                f"(round {round_no}, t={t_global}): livelock?")

        # sanity sweep: a displaced mid-action agent (no longer beside its
        # task cell) must abandon the action, or it squats other targets
        # while being unevictable ("mid-action" protection) -> deadlock
        for a in agents:
            a.action_age = a.action_age + 1 if a.action is not None else 0
            if a.action in ("place", "remove") and a.task is None:
                a.action = None  # zombie: action without a task
            elif a.action in ("place", "remove") and a.task is not None:
                tr_, tc_ = a.task.cell
                if abs(a.pos[0] - tr_) + abs(a.pos[1] - tc_) != 1:
                    a.action = None
                    a.task = None
                    a.action_fails = 0
            elif a.action == "pick" and a.pos not in DEPOT:
                a.action = None
            elif a.action == "deposit" and a.pos not in DEPOT:
                a.action = None
            if a.action is not None and a.action_age > 20:
                # stuck pending action: abandon it, cool the task down and
                # let the global replan route around the jam
                if a.task is not None:
                    task_cooldown[(a.task.cell, a.task.level, a.task.kind)] = round_no + 30
                a.action = None
                a.task = None
                a.action_fails = 0
                a.action_age = 0

        # ---------------- offers: candidate goals per agent ----------------
        # reachability via connected-component labels; full Dijkstra only on
        # demand (depot ranking, hold-cell choice), both cached while the
        # terrain is unchanged (rounds ending in pick/deposit keep the cache)
        cache_ver = len(terrain_events)
        if cache_ver != dist_cache_ver[0]:
            dist_cache.clear()
            comp_cache.clear()
            dist_cache_ver[0] = cache_ver
        if not comp_cache:
            comp_cache.append(component_labels(heights))
        comp = comp_cache[0]

        def dist_from(pos: Coord) -> List[List[int]]:
            if pos not in dist_cache:
                dist_cache[pos] = dijkstra(pos, heights)
            return dist_cache[pos]

        def reachable(a: Agent, cell: Coord) -> bool:
            return comp[a.pos[0]][a.pos[1]] == comp[cell[0]][cell[1]]

        def _cool(t: Task) -> bool:
            return task_cooldown.get((t.cell, t.level, t.kind), 0) > round_no
        ready_pl = [t for t in build_pending if not _cool(t) and ready_place(t)]
        ready_rm = [t for t in demolition if not _cool(t) and ready_remove(t)] if removal_latch else []

        # map stand cell -> task (first task claims the cell); never offer a
        # stand cell that is itself the target of another ready task, or two
        # agents can block each other's placements forever
        # standing on another ready task's target is allowed only toward a
        # strictly later task in a fixed total order: chains cannot cycle, so
        # the "conga line" standoff (everyone on someone else's target) is
        # impossible; deck targets stay fully banned as stands
        def tkey(t: Task) -> tuple:
            return (t.level, t.cell)

        def sel_key(t: Task) -> tuple:
            # placement grows bottom-up (low level first); demolition strips
            # top-down (HIGH level first), or access stairs vanish under the
            # crew and elevated scaffold becomes an unreachable island
            return (t.level, t.cell) if t.kind == "place" else (-t.level, t.cell)

        target_key = {}
        for t in ready_pl + ready_rm:
            if deck_task(t.cell, t.level):
                target_key[t.cell] = None  # never stand here
            else:
                target_key[t.cell] = tkey(t)

        # tasks already being executed by a mid-action agent are off the
        # market: offering their other stand cells would double-book them
        active_tasks = {(a.task.cell, a.task.level, a.task.kind)
                        for a in agents
                        if a.action is not None and a.task is not None}

        # one canonical stand cell per task (per reachability class): a task
        # maps to exactly one goal cell, so the assignment can never park a
        # second agent on a redundant stand (which may be another task's
        # target and would block its placement); surplus agents hold instead
        def task_stand_candidates(tsk: Task) -> List[Coord]:
            cands = []
            for n in task_stand_cells(tsk):
                if n in target_key:
                    nk = target_key[n]
                    if nk is None or nk <= tkey(tsk):
                        continue
                cands.append(n)
            # prefer stands that are not themselves ready-task targets
            cands.sort(key=lambda n: (n in target_key, n))
            return cands

        task_stands: List[Tuple[Task, List[Coord]]] = []
        for tsk in ready_pl + ready_rm:
            if (tsk.cell, tsk.level, tsk.kind) in active_tasks:
                continue
            cands = task_stand_candidates(tsk)
            if cands:
                task_stands.append((tsk, cands))

        # escalated anti-jam: if the level-1 throttle didn't clear the jam,
        # offer ONE task globally (works for face-to-face deadlocks on the
        # structure itself, not just on scaffold corridors)
        if progress_stall >= 2 * THROTTLE_AT and cell2task:
            best_one = min(cell2task.items(), key=lambda kv: sel_key(kv[1]))
            cell2task = dict([best_one])

        carrying_new = sum(1 for a in agents if (a.carrying and a.cargo == "new") or a.action == "pick")
        pick_quota = remaining_places - carrying_new
        action_cells = {a.pos for a in agents if a.action is not None}
        hold_positions = set()

        avail_depot = [d for d in DEPOT if d not in action_cells]
        # canonical stand assignment: each task gets exactly one stand cell,
        # chosen nearest to its eligible agents (carriers for place, free
        # agents for remove); greedy in deterministic task order
        carriers_pos = [a.pos for a in agents
                        if a.action is None and a.carrying and a.cargo == "new"]
        free_pos = [a.pos for a in agents
                    if a.action is None and not a.carrying]

        def stand_rank(n: Coord, tsk: Task) -> Tuple[int, int, Coord]:
            srcs = carriers_pos if tsk.kind == "place" else free_pos
            d = min((dist_from(p)[n[0]][n[1]] for p in srcs), default=INF)
            return (n in target_key, d, n)  # off-target first, then nearest

        cell2task: Dict[Coord, Task] = {}
        for tsk, cands in sorted(task_stands, key=lambda x: sel_key(x[0])):
            ranked = sorted(cands, key=lambda n: stand_rank(n, tsk))
            for n in ranked:
                if n not in cell2task and n not in action_cells:
                    cell2task[n] = tsk
                    break
        place_cells = {n: t for n, t in cell2task.items() if t.kind == "place"}
        remove_cells = {n: t for n, t in cell2task.items() if t.kind == "remove"}

        # 1-wide elevated scaffolds (stairs/walls) are single-file corridors:
        # expose at most ONE task whose stand sits on each scaffold component,
        # or transiting workers livelock on each other's target cells
        scaffold_set = set(SCAFFOLD_CELLS)
        if scaffold_set:
            comp_id: Dict[Coord, int] = {}
            for cell in scaffold_set:
                if cell in comp_id:
                    continue
                cid = len(comp_id)
                stack = [cell]
                comp_id[cell] = cid
                while stack:
                    r0_, c0_ = stack.pop()
                    for dr, dc in NBRS:
                        nb = (r0_ + dr, c0_ + dc)
                        if nb in scaffold_set and nb not in comp_id:
                            comp_id[nb] = comp_id[cell]
                            stack.append(nb)
            comp_size: Dict[int, int] = {}
            comp_cells: Dict[int, list] = {}
            for cell, g in comp_id.items():
                comp_size[g] = comp_size.get(g, 0) + 1
                comp_cells.setdefault(g, []).append(cell)
            # per-component single-file granularity: corridors WITH a cycle
            # (2-core non-empty) let workers pass around -> dense segments;
            # pure chains support one worker per passing pocket plus one
            seg_idx: Dict[Coord, int] = {}
            for g, members in comp_cells.items():
                if comp_size[g] < SCAFFOLD_MIN:
                    continue  # short stairs: no single-file restriction
                deg = {x: sum(1 for dr, dc in NBRS
                              if comp_id.get((x[0] + dr, x[1] + dc)) == g)
                       for x in members}
                core_deg = dict(deg)
                core = set(members)
                peel = [x for x in members if core_deg[x] <= 1]
                while peel:
                    x = peel.pop()
                    if x not in core:
                        continue
                    core.discard(x)
                    for dr, dc in NBRS:
                        nb = (x[0] + dr, x[1] + dc)
                        if nb in core:
                            core_deg[nb] -= 1
                            if core_deg[nb] <= 1:
                                peel.append(nb)
                seg_len = SCAFFOLD_SEG
                start = min(members, key=lambda x: (deg[x], x))
                order_q = [start]
                seen_c = {start}
                idx = 0
                while order_q:
                    cur = order_q.pop(0)
                    seg_idx[cur] = idx // seg_len
                    idx += 1
                    for dr, dc in NBRS:
                        nb = (cur[0] + dr, cur[1] + dc)
                        if comp_id.get(nb) == g and nb not in seen_c:
                            seen_c.add(nb)
                            order_q.append(nb)
            best: Dict[Tuple[int, int], Tuple[tuple, Coord]] = {}
            for n, tsk in cell2task.items():
                if n in scaffold_set and comp_size[comp_id[n]] >= SCAFFOLD_MIN:
                    key = (comp_id[n], seg_idx.get(n, 0))
                    if key not in best or sel_key(tsk) < best[key][0]:
                        best[key] = (sel_key(tsk), n)
            if progress_stall >= THROTTLE_AT:
                throttle_until[0] = round_no + THROTTLE_HOLD
            if round_no < throttle_until[0]:
                # anti-jam throttle with hysteresis: ONE task per corridor
                one: Dict[int, Tuple[tuple, Coord]] = {}
                for (g, _seg), (k, n) in best.items():
                    if g not in one or k < one[g][0]:
                        one[g] = (k, n)
                best = {(g, 0): v for g, v in one.items()}
            keep = {v[1] for v in best.values()}

            def allowed_stand(n: Coord) -> bool:
                if n not in scaffold_set or comp_size[comp_id[n]] < SCAFFOLD_MIN:
                    return True  # short stairs: no single-file restriction
                return n in keep

            place_cells = {n: t for n, t in place_cells.items() if allowed_stand(n)}
            remove_cells = {n: t for n, t in remove_cells.items() if allowed_stand(n)}

        # task urgency: place = blocks still to stack above; remove = scaffold
        # levels still to strip.  deeper chain = more urgent = cheaper goal
        def task_urgency(t: Task) -> int:
            if t.kind == "place":
                return max(T_BUILD.get(t.cell, t.level) - t.level, 0)
            return max(t.level - T_FINAL.get(t.cell, 0), 0)

        u_max = max((task_urgency(t) for t in cell2task.values()), default=0)

        # depot offers stay quota-limited (business rule: never pick more
        # blocks than there are placements left), nearest agents first
        free_agents = sorted(
            (a for a in agents if not a.carrying and a.action is None),
            key=lambda a: min((dist_from(a.pos)[d[0]][d[1]] for d in avail_depot), default=INF))
        depot_agents = {a.name for a in
                        free_agents[:min(max(pick_quota, 0), len(avail_depot))]}

        # feasibility is guaranteed by the solver-side hold goal (appended
        # below with HOLD_COST): no capacity trimming, no matching pre-check
        offers: Dict[str, Dict[Coord, Tuple[str, Optional[Task]]]] = {}
        offer_costs: Dict[str, Dict[Coord, int]] = {}
        for a in agents:
            offer: Dict[Coord, Tuple[str, Optional[Task]]] = {}
            cost: Dict[Coord, int] = {}

            def add(cell: Coord, intent: str, tsk: Optional[Task], c: int) -> None:
                if cell not in offer:
                    offer[cell] = (intent, tsk)
                    cost[cell] = c

            def sticky(cell: Coord) -> int:
                return STICKY_BONUS if (a.prev_goal is not None
                                        and cell != a.prev_goal) else 0

            if a.action is not None:
                add(a.pos, "action", None, 0)
            elif a.carrying and a.cargo == "scrap":
                for dep in avail_depot:
                    if reachable(a, dep):
                        add(dep, "return", None, sticky(dep))
            elif a.carrying and a.cargo == "new":
                for n, tsk in place_cells.items():
                    if reachable(a, n):
                        add(n, "stand", tsk,
                            PRIO_W * (u_max - task_urgency(tsk)) + sticky(n))
            else:
                for n, tsk in remove_cells.items():
                    if reachable(a, n):
                        add(n, "stand", tsk,
                            PRIO_W * (u_max - task_urgency(tsk)) + sticky(n))
                if a.name in depot_agents:
                    for dep in avail_depot:
                        if reachable(a, dep):
                            add(dep, "depot", None, sticky(dep))
            offers[a.name] = offer
            offer_costs[a.name] = cost

        # evicted agents (they blocked someone's action) must move: never
        # offer them their own position
        for a in agents:
            if a.name in evict_names and a.action is None:
                offers[a.name].pop(a.pos, None)
                offer_costs[a.name].pop(a.pos, None)
        evicted_now = {a.name for a in agents if a.name in evict_names and a.action is None}
        evict_names.clear()

        # hold-as-goal: every non-action agent gets one distinct fallback
        # cell (off work cells, off cells offered to anyone) at HOLD_COST;
        # the solver's assignment decides who actually holds
        bad = work_cells()
        offered_cells = set()
        for od in offers.values():
            offered_cells |= set(od.keys())
        for a in agents:
            if a.action is not None:
                continue
            if (a.pos not in bad and a.pos not in offered_cells
                    and a.name not in evicted_now):
                hold = a.pos
            else:
                d = dist_from(a.pos)
                free = [(d[r][c], (r, c)) for r in range(ROWS) for c in range(COLS)
                        if d[r][c] < INF and (r, c) not in bad
                        and (r, c) not in offered_cells and (r, c) not in hold_positions]
                if not free:
                    hold = a.pos  # temporarily trapped: wait for terrain to change
                else:
                    hold = min(free)[1]
            if hold not in offers[a.name]:
                offers[a.name][hold] = ("hold", None)
                # a lone hold goal costs 0: no cheaper option should exist
                offer_costs[a.name][hold] = HOLD_COST if len(offers[a.name]) > 1 else 0
            hold_positions.add(hold)
            offered_cells.add(hold)

        # ---------------- solve: LaCAM-TAPF does the matching ----------------
        if KEEP_ROUNDS > 0:
            rd = WORK / f"round_{round_no:04d}"
            stale = WORK / f"round_{round_no - KEEP_ROUNDS:04d}"
            if stale.is_dir():
                shutil.rmtree(stale, ignore_errors=True)
        else:
            rd = WORK / "current"  # non-debug: reuse one directory
        rd.mkdir(parents=True, exist_ok=True)
        map_path = rd / "round.map"
        write_map(map_path)
        agents_yaml = [{"name": a.name, "start": [a.pos[0], a.pos[1]],
                        "potentialGoals": [[g[0], g[1]] for g in offers[a.name]],
                        "goalCosts": [offer_costs[a.name][g] for g in offers[a.name]]}
                       for a in agents]
        in_yaml = rd / "in.yaml"
        in_yaml.write_text(yaml.safe_dump({
            "map": str(map_path.resolve()),
            "heights": [list(row) for row in heights],
            "climbCost": CLIMB_COST,
            "agents": agents_yaml,
        }, sort_keys=False), encoding="utf-8")
        out_yaml = rd / "out.yaml"
        cmd = [str(BINARY), str(in_yaml), "", str(TIME_LIMIT), str(out_yaml),
               SOLVER_ANYTIME, "0", "0", SOLVER_MODE, SOLVER_FOCAL_W, SOLVER_TIE]
        stdout = ""
        solved = False
        for attempt in range(4):  # timed-out rounds: retry with fresh seeds
            cmd[7] = str(attempt - 1)  # -1, 0, 1, 2
            proc = subprocess.run(cmd, capture_output=True, text=True,
                                  timeout=TIME_LIMIT + 10)
            stdout = proc.stdout or ""
            if proc.returncode == 0 and "solved=1" in stdout:
                solved = True
                break
            print(f"round {round_no}: solver attempt {attempt} failed "
                  f"(rc={proc.returncode}, timed_out="
                  f"{'timed_out=1' in stdout}), retrying with new seed",
                  flush=True)
        if not solved:
            # freeze round: demote every non-action agent to a pure hold and
            # solve the trivial instance; pending actions complete, terrain
            # changes, and the next round re-offers everything
            print(f"round {round_no}: unsolvable offer set, freezing round "
                  f"(agents hold, pending actions run)", flush=True)
            for a in agents:
                if a.action is None:
                    hold = next(iter(offers[a.name]))  # existing hold last
                    for g, (intent, _) in offers[a.name].items():
                        if intent == "hold":
                            hold = g
                    offers[a.name] = {hold: ("hold", None)}
                    offer_costs[a.name] = {hold: 0}
            agents_yaml = [{"name": a.name, "start": [a.pos[0], a.pos[1]],
                            "potentialGoals": [[g[0], g[1]] for g in offers[a.name]],
                            "goalCosts": [offer_costs[a.name][g] for g in offers[a.name]]}
                           for a in agents]
            in_yaml.write_text(yaml.safe_dump({
                "map": str(map_path.resolve()),
                "heights": [list(row) for row in heights],
                "climbCost": CLIMB_COST,
                "agents": agents_yaml,
            }, sort_keys=False), encoding="utf-8")
            proc = subprocess.run(cmd, capture_output=True, text=True,
                                  timeout=TIME_LIMIT + 10)
            stdout = proc.stdout or ""
            if proc.returncode != 0 or "solved=1" not in stdout:
                raise RuntimeError(
                    f"solver failed round {round_no} even frozen:\n{stdout}\n{proc.stderr}")

        data = load_schedule(out_yaml)
        makespan = max(int(data["statistics"]["makespan"]), 1)
        sched = data["schedule"]
        paths = [expand_schedule(sched.get(f"agent{i}", []), agents[i].pos, makespan)
                 for i in range(len(agents))]

        # recover assignment: goal cell -> intent/task
        assignments = data.get("assignments") or {}
        used_tasks: set = set()
        for a in agents:  # mid-action agents claim their tasks first
            if a.action is not None and a.task is not None:
                used_tasks.add(id(a.task))
        for i, a in enumerate(agents):
            if a.action is not None:  # mid-action: keep task/goal untouched
                a.goal = a.pos
                a.intent = "action"
                continue
            node = assignments.get(f"agent{i}")
            goal = (int(node["x"]), int(node["y"])) if node else paths[i][-1]
            intent, task = offers[a.name].get(goal, ("hold", None))
            if task is not None and id(task) in used_tasks:
                intent, task = "hold", None  # duplicate stand for same task
            if task is not None:
                used_tasks.add(id(task))
            a.goal = goal
            a.intent = intent
            a.task = task
            if TRACE_A2 and a.name in ("a2", "a7") and round_no % 50 == 0:
                print(f"TRACE r{round_no} {a.name} pos={a.pos} act={a.action} fails={a.action_fails} intent={intent} goal={goal} task={task.cell if task else None} evict={a.name in evict_names}", flush=True)
            # stickiness: remember real assignments (not holds) across rounds
            a.prev_goal = goal if intent in ("stand", "depot", "return") else None

        # ---------------- action initiation / completion ----------------
        def start_action_if_arrived(a: Agent) -> bool:
            if a.action is not None or a.pos != a.goal:
                return False
            if a.name in evict_names:
                return False  # must vacate first; replan will move it
            if a.intent == "depot" and not a.carrying:
                a.action = "pick"
                return True
            if a.intent == "stand" and a.task is not None:
                if a.task.kind == "place" and a.carrying:
                    a.action = "place"
                    return True
                if a.task.kind == "remove" and not a.carrying:
                    a.action = "remove"
                    return True
            if a.intent == "return" and a.carrying and a.cargo == "scrap":
                a.action = "deposit"
                return True
            return False

        def complete_action(a: Agent, positions: List[Coord]) -> bool:
            nonlocal remaining_places
            act, a.action = a.action, None
            if act == "pick":
                a.action_fails = 0
                a.carrying = True
                a.cargo = "new"
                a.carry_hist[-1] = 1
                pick_events.append({"t": t_global, "agent": int(a.name[1:]),
                                    "r": a.pos[0], "c": a.pos[1]})
                return True
            if act == "place" and a.task is not None:
                tc = a.task.cell
                deck = deck_task(tc, a.task.level)
                if (tc in positions or not ready_place(a.task)
                        or (not deck and heights[tc[0]][tc[1]] != a.task.level - 1)):
                    import os as _os
                    if _os.environ.get("DBG_FAIL"):
                        print(f"   FAIL place {tc}@{a.task.level} by {a.name}@{a.pos}: "
                              f"occ={tc in positions} ready={ready_place(a.task)} "
                              f"h={heights[tc[0]][tc[1]]}", flush=True)
                    if tc in positions:
                        for b in agents:
                            if b.pos == tc and b.action is None:
                                evict_names.add(b.name)
                        a.action_fails += 1
                        if ready_place(a.task) and a.action_fails <= 8:
                            a.action = act  # transient blocker: retry next step
                        else:
                            # persistent blocker (likely trapped): cool the
                            # task down so the corridor can clear
                            task_cooldown[(a.task.cell, a.task.level, a.task.kind)] = round_no + 30
                    return False
                if deck:
                    assert heights[a.pos[0]][a.pos[1]] in (a.task.level, a.task.level + 1)
                    heights[tc[0]][tc[1]] = a.task.level
                else:
                    assert heights[a.pos[0]][a.pos[1]] in (a.task.level - 1, a.task.level)
                    heights[tc[0]][tc[1]] += 1
                peaks[tc[0]][tc[1]] = max(peaks[tc[0]][tc[1]], heights[tc[0]][tc[1]])
                terrain_events.append({"t": t_global, "r": tc[0], "c": tc[1],
                                       "h": heights[tc[0]][tc[1]], "agent": int(a.name[1:])})
                build_pending.remove(a.task)
                a.task = None
                a.carrying = False
                a.cargo = None
                a.carry_hist[-1] = 0
                remaining_places -= 1
                a.action_fails = 0
                return True
            if act == "remove" and a.task is not None:
                tc = a.task.cell
                if (tc in positions or heights[tc[0]][tc[1]] != a.task.level
                        or not ready_remove(a.task)):
                    import os as _os
                    if _os.environ.get("DBG_FAIL"):
                        print(f"   FAILrm {tc}@{a.task.level} by {a.name}@{a.pos}: "
                              f"occ={tc in positions} rdy={ready_remove(a.task)} "
                              f"h={heights[tc[0]][tc[1]]}", flush=True)
                    if tc in positions:
                        for b in agents:
                            if b.pos == tc and b.action is None:
                                evict_names.add(b.name)  # idle blocker: move
                            # mid-action blockers are left alone: they finish
                            # their own removal next step and walk away
                        a.action_fails += 1
                        if ready_remove(a.task) and a.action_fails <= 8:
                            a.action = act  # transient blocker: retry next step
                        else:
                            task_cooldown[(a.task.cell, a.task.level, a.task.kind)] = round_no + 30
                    return False
                assert heights[a.pos[0]][a.pos[1]] in (a.task.level - 1, a.task.level)
                heights[tc[0]][tc[1]] -= 1
                terrain_events.append({"t": t_global, "r": tc[0], "c": tc[1],
                                       "h": heights[tc[0]][tc[1]], "agent": int(a.name[1:])})
                demolition.remove(a.task)
                a.task = None
                a.carrying = True
                a.cargo = "scrap"
                a.carry_hist[-1] = 1
                nonlocal demolition_start_t
                if demolition_start_t == 0:
                    demolition_start_t = t_global
                a.action_fails = 0
                return True
            if act == "deposit":
                a.carrying = False
                a.cargo = None
                a.carry_hist[-1] = 0
                deposit_events.append({"t": t_global, "agent": int(a.name[1:]),
                                       "r": a.pos[0], "c": a.pos[1]})
                a.action_fails = 0
                return True
            return False

        # zero-time: agents already at their assigned goal begin acting
        zero = False
        for a in agents:
            if start_action_if_arrived(a):
                zero = True
        if zero:
            stall_rounds = 0
            continue

        # ---------------- validate solver paths ----------------
        for i, p in enumerate(paths):
            assert p[0] == agents[i].pos, f"round {round_no}: bad start {i}"
            for t in range(1, len(p)):
                (r0, c0), (r1, c1) = p[t - 1], p[t]
                assert abs(r1 - r0) + abs(c1 - c0) <= 1, f"round {round_no}: non-unit move"
                assert abs(heights[r1][c1] - heights[r0][c0]) <= 1, f"round {round_no}: height jump"
        for t in range(makespan + 1):
            seen = set()
            for p in paths:
                pos = p[min(t, len(p) - 1)]
                assert pos not in seen, f"round {round_no}: vertex collision t={t}"
                seen.add(pos)
        for t in range(makespan):
            for i in range(len(paths)):
                for j in range(i + 1, len(paths)):
                    if paths[i][t] == paths[j][t + 1] and paths[i][t + 1] == paths[j][t]:
                        raise AssertionError(f"round {round_no}: edge collision t={t}")

        # ---------------- execute ----------------
        t_round_start = t_global
        moved = False
        event_this_round = False
        completed_this_round = False
        started = {a.name for a in agents if a.action is not None}
        for t in range(1, makespan + 1):
            for i, a in enumerate(agents):
                if a.pos != paths[i][t]:
                    moved = True
                a.pos = paths[i][t]
                a.path_hist.append(a.pos)
                a.carry_hist.append(1 if a.carrying else 0)
            t_global += 1
            positions = [a.pos for a in agents]
            for a in agents:
                if a.action is not None and a.path_hist[-1] != a.path_hist[-2]:
                    a.action = None  # displaced (even mid-round): abort action
                    continue
                if a.action is None or a.name not in started:
                    continue
                if complete_action(a, positions):
                    event_this_round = True
                    completed_this_round = True
            for a in agents:
                if start_action_if_arrived(a):
                    event_this_round = True
            if event_this_round:
                break

        # ---------------- per-round rule checks (validate_plan.py rules) ----
        depot_set = set(DEPOT)
        for ev in terrain_events:
            if not (t_round_start < ev["t"] <= t_global):
                continue
            t, r, c = ev["t"], ev["r"], ev["c"]
            actor = agents[ev["agent"]]
            assert all(tuple(b.path_hist[t]) != (r, c) for b in agents), \
                f"round {round_no}: terrain event on occupied cell t={t} ({r},{c})"
            ar, ac = actor.path_hist[t]
            assert abs(ar - r) + abs(ac - c) == 1, \
                f"round {round_no}: acting agent not adjacent t={t} ({r},{c})"
        for ev in pick_events:
            if not (t_round_start < ev["t"] <= t_global):
                continue
            t = ev["t"]
            actor = agents[ev["agent"]]
            assert (ev["r"], ev["c"]) in depot_set, f"round {round_no}: pick off depot {ev}"
            assert actor.path_hist[t] == (ev["r"], ev["c"]), f"round {round_no}: pick position {ev}"
            assert actor.carry_hist[t - 1] == 0 and actor.carry_hist[t] == 1, \
                f"round {round_no}: pick carry not 0->1 {ev}"
        for ev in deposit_events:
            if not (t_round_start < ev["t"] <= t_global):
                continue
            t = ev["t"]
            actor = agents[ev["agent"]]
            assert (ev["r"], ev["c"]) in depot_set, f"round {round_no}: deposit off depot {ev}"
            assert actor.path_hist[t] == (ev["r"], ev["c"]), f"round {round_no}: deposit position {ev}"
            assert actor.carry_hist[t - 1] == 1 and actor.carry_hist[t] == 0, \
                f"round {round_no}: deposit carry not 1->0 {ev}"

        stall_rounds = 0 if (moved or event_this_round) else stall_rounds + 1
        progress_stall = 0 if completed_this_round else progress_stall + 1
        import os
        if os.environ.get("DBG_ROUND") and round_no >= int(os.environ["DBG_ROUND"]):
            rp = [t for t in build_pending if ready_place(t)]
            print('READY:', [(t.cell, t.level) for t in rp][:12], flush=True)
            for rr in range(3, 12):
                print('   ', ''.join(str(min(heights[rr][cc], 9)) for cc in range(4, 17)), flush=True)
            for a in agents:
                print(f'   {a.name} pos={a.pos} h={heights[a.pos[0]][a.pos[1]]} carry={a.carrying} intent={a.intent} goal={a.goal} task={(a.task.cell, a.task.level) if a.task else None}', flush=True)
            print('   DBG place_cells:', {k: (v.cell, v.level) for k, v in list(place_cells.items())[:8]}, flush=True)
            print('   DBG offers:', {a.name: {g: offers[a.name][g][0] for g in offers[a.name]} for a in agents}, flush=True)
            rr_ = [t for t in demolition if ready_remove(t)]
            print('   DBG latch:', removal_latch, 'ready_rm:', [(t.cell, t.level) for t in rr_][:8], flush=True)
            print('   DBG remove_cells:', {k: (v.cell, v.level) for k, v in list(remove_cells.items())[:8]}, flush=True)
            print('   DBG ready_pl:', [(t.cell, t.level) for t in ready_pl][:10], flush=True)
            for rr2 in (0, 1, 2, min(12, ROWS-1), min(13, ROWS-1), min(14, ROWS-1)):
                print('   row', rr2, [heights[rr2][cc] for cc in range(2, 20)], flush=True)
            import json as _json
            _json.dump({"h": heights, "tb": {str(k): v for k, v in T_BUILD.items()},
                        "dl": {str(k): v for k, v in DECK_LEVEL.items()}}, open("/tmp/hstate.json", "w"))
            for rr3 in range(3, 11):
                print('   r%02d' % rr3, ''.join(str(min(heights[rr3][cc], 9)) for cc in range(5, 19)), flush=True)
            for t in build_pending:
                if t.level - 1 == heights[t.cell[0]][t.cell[1]] and t.level <= 4:
                    print(f'   frontier {t.cell}@{t.level} anc={anchored_after(t.cell, t.level)} ready={ready_place(t)} stands={task_stand_cells(t)}', flush=True)
            for t in build_pending:
                if t.cell == (4, 8) and t.level == 2:
                    print('   (4,8)@2 sup:', heights[4][8] == 1,
                          'anc:', anchored_after((4, 8), 2),
                          'ready:', ready_place(t), flush=True)
            print('   ring row4:', [heights[4][cc] for cc in range(6, 16)], flush=True)
            print('   ring row9:', [heights[9][cc] for cc in range(6, 16)], flush=True)
            print('   tails:', heights[10][10], heights[11][10], heights[12][10], '|', heights[3][10], heights[2][10], heights[1][10], flush=True)
            for t in build_pending:
                if t.cell in ((10,10),(11,10),(2,10),(3,10),(4,7),(4,10)) and t.level <= 3:
                    sup = heights[t.cell[0]][t.cell[1]] == t.level - 1
                    anc = anchored_after(t.cell, t.level) if sup else None
                    print(f'   pend {t.cell}@{t.level} sup={sup} anc={anc} ready={ready_place(t)}', flush=True)
            tails_all = [t for t in build_pending if t.cell[0] in (0,1,2,12,13,14) or t.cell[1] in (2,3,4,16,17,18)]
            for s in tails_all[:8]:
                sup = heights[s.cell[0]][s.cell[1]] == s.level - 1
                anc = anchored_after(s.cell, s.level)
                print(f'   tail-task {s.cell}@{s.level} support={sup} anchored={anc}', flush=True)
            raise SystemExit(0)
        print(f"round {round_no:4d}  t={t_global:5d}  placed {total_places - remaining_places}/{total_places}"
              f"  demolition left {len(demolition)}", flush=True)

    assert remaining_places == 0 and not demolition, "column build incomplete"
    expect = [[0] * COLS for _ in range(ROWS)]
    for (r, c), h in T_FINAL.items():
        expect[r][c] = h
    assert heights == expect, "final terrain mismatch"

    plan = {
        "rows": ROWS,
        "cols": COLS,
        "T": t_global,
        "depot": [list(d) for d in DEPOT],
        "meta": {"statsLine": f"柱子高 {COL_H} · 脚手架 {sum(T_BUILD[c] for c in SCAFFOLD_CELLS)} 块（用完拆除）",
                 "demolitionStart": demolition_start_t,
                 "suspenders": SUSPENDERS,
                 "struts": STRUTS},
        "final_heights": expect,
        "colors": {f"{r},{c}": col for (r, c), col in COLORS.items()},
        "colors3": {f"{r},{c},{h}": col for (r, c, h), col in COLORS3.items()},
        "floating": {f"{r},{c}": z for (r, c), z in DECK_LEVEL.items()},
        "agents": [{"name": a.name, "path": [list(p) for p in a.path_hist],
                    "carry": a.carry_hist} for a in agents],
        "terrain": terrain_events,
        "picks": pick_events,
        "deposits": deposit_events,
    }
    OUT_JSON.write_text(json.dumps(plan), encoding="utf-8")
    OUT_JS.write_text("const PLAN = " + json.dumps(plan) + ";\n", encoding="utf-8")
    n_removes = len(terrain_events) - total_places
    print(f"done: T={t_global}, rounds={round_no}, places={total_places}, "
          f"removals={n_removes}, deposits={len(deposit_events)}")
    print(f"wrote {OUT_JSON} and {OUT_JS}")
    # full-tape rule validation (validate_plan.py) as the final gate
    vp = Path(__file__).resolve().parent / "validate_plan.py"
    rc = subprocess.run([sys.executable, str(vp), str(OUT_JSON)]).returncode
    if rc != 0:
        raise RuntimeError("validate_plan.py failed on the generated plan")
    return 0


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--col-h", type=int, default=6)
    ap.add_argument("--helper", action="store_true",
                    help="parallel helper lane one level lower, for faster demolition")
    ap.add_argument("--agents", type=int, default=6)
    ap.add_argument("--scene", action="store_true", help="Giza scene: 2 pyramids + obelisk")
    ap.add_argument("--bridge", action="store_true", help="arch bridge with side-adhesion deck")
    ap.add_argument("--temple", action="store_true", help="Parthenon-style temple with hollow roof")
    ap.add_argument("--colonnade", action="store_true", help="colonnade temple: pillars + floating roof")
    ap.add_argument("--horse", action="store_true", help="Trojan horse with floating belly and head")
    ap.add_argument("--pyramid", action="store_true", help="plain pyramid via unified planner")
    ap.add_argument("--goldengate", action="store_true", help="Golden Gate bridge in international orange")
    ap.add_argument("--uscgate", action="store_true", help="USC gate with gold USC ground inlay")
    ap.add_argument("--symbot", action="store_true", help="Symbotic-style bot with floating chassis")
    ap.add_argument("--tommy", action="store_true", help="Tommy Trojan statue with sword and shield")
    ap.add_argument("--exp", action="store_true", help="parametric scaffold research scene (env-driven)")
    ap.add_argument("--out-prefix", type=str, default="plan_column")
    args = ap.parse_args()
    if args.scene:
        configure_scene()
    elif args.bridge:
        configure_bridge()
        set_agents(args.agents if args.agents != 6 else 8)
    elif args.temple:
        configure_temple()
    elif args.colonnade:
        configure_colonnade()
    elif args.horse:
        configure_horse()
    elif args.pyramid:
        configure_pyramid()
    elif args.goldengate:
        configure_goldengate()
    elif args.uscgate:
        configure_uscgate()
    elif args.symbot:
        configure_symbot()
    elif args.tommy:
        configure_tommy()
    elif args.exp:
        configure_exp()
    else:
        configure(args.col_h, args.helper)
        set_agents(args.agents)
    base = Path(__file__).resolve().parent
    OUT_JSON = base / f"{args.out_prefix}.json"
    OUT_JS = base / f"{args.out_prefix}.js"
    WORK = base / f"work_{args.out_prefix}"
    sys.exit(main())
