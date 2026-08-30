#!/usr/bin/env python3
"""design.md 8.2 sweep-axis instance suite (debug.md task 13).

Axes (2 seeds each, 12x12 scrambler base unless noted):
  ratio  robot:shelf in {1:2, 1:5, 1:10, 1:20}   (shelves fixed at 40)
         plus 1:50 on a 40x40 map (200 shelves, 4 robots) — the 12x12 map
         cannot hold 50 shelves per robot (design 8.2 tier, round-2 P1-10)
  fill   shelf fill rate in {50, 70, 85, 95}%     (4 robots, t=8)
  ntgt   |B_tgt| in {1, 4, 16, 64}                (fill 50%, 6 robots)
  depth  scramble k in {40, 160, 640}             (fill 30%, 4 robots)

gamma/overhead axis needs no reruns: counters (loaded/free/liftdrop/anon)
are recorded per row, so weighted SOC under any (a,b,g,d) profile is a
post-hoc reweighting.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from ddbench.generators import scramble_with_witness
from ddbench.instance import save_instance
from ddbench.validator import validate_plan

OUT = Path(__file__).parent / "instances_sweep"
SEEDS = 2


def gen(dirname, **kw):
    d = OUT / dirname
    d.mkdir(parents=True, exist_ok=True)
    for seed in range(SEEDS):
        ins, witness = scramble_with_witness(seed=seed, **kw)
        ok, errs, _ = validate_plan(ins, witness)
        assert ok, errs
        save_instance(ins, d / f"{ins.name}.yaml")


def main():
    n = 0
    # ratio axis: 40 shelves, robots 20/8/4/2
    for r in (20, 8, 4, 2):
        gen(f"ratio_r{r}", height=12, width=12, n_robots=r, n_shelves=40,
            n_targets=8, k=160)
        n += SEEDS
    # 1:50 tier needs a bigger map: 40x40 (1600 cells), 4 robots, 200
    # shelves (12.5% fill), 8 targets
    gen("ratio_r4x50", height=40, width=40, n_robots=4, n_shelves=200,
        n_targets=8, k=320)
    n += SEEDS
    # fill axis: 144 cells -> shelves 72/100/122/136
    for fill, s in ((50, 72), (70, 100), (85, 122), (95, 136)):
        gen(f"fill_{fill}", height=12, width=12, n_robots=4, n_shelves=s,
            n_targets=8, k=160)
        n += SEEDS
    # |B_tgt| axis
    for t in (1, 4, 16, 64):
        gen(f"ntgt_{t}", height=12, width=12, n_robots=6, n_shelves=72,
            n_targets=t, k=160)
        n += SEEDS
    # scramble depth axis
    for k in (40, 160, 640):
        gen(f"depth_{k}", height=12, width=12, n_robots=4, n_shelves=43,
            n_targets=8, k=k)
        n += SEEDS
    print(f"generated {n} sweep instances under {OUT}/")


if __name__ == "__main__":
    main()
