#!/usr/bin/env python3
"""Generate the dd-lacam benchmark instance set (design.md 8.2).

Two families:
  scramble/  — scrambler protocol (feasible by construction), difficulty k
  ddmapd/    — DD-MAPD paper protocol (2x2 blocks, perimeter agents)

Each instance is written as YAML; scramble instances also get a witness plan
sanity check through the validator before being accepted.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from ddbench.generators import ddmapd_instance, scramble_with_witness
from ddbench.instance import save_instance
from ddbench.validator import validate_plan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default="instances")
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--small", action="store_true",
                    help="small suite (NAT-CBS-compatible sizes only)")
    args = ap.parse_args()
    out = Path(args.out_dir)
    (out / "scramble").mkdir(parents=True, exist_ok=True)
    (out / "ddmapd").mkdir(parents=True, exist_ok=True)

    count = 0

    # ---- scramble family: grid, robots, shelves, targets, k ----
    if args.small:
        scramble_grid = [
            # (h, w, robots, shelves, targets, k)
            (6, 6, 1, 4, 2, 20),
            (6, 6, 2, 4, 2, 20),
            (6, 6, 2, 8, 2, 40),
            (8, 8, 2, 8, 4, 40),
            (8, 8, 4, 12, 4, 80),
        ]
    else:
        scramble_grid = [
            (6, 6, 2, 8, 2, 20),
            (8, 8, 2, 12, 4, 40),
            (8, 8, 4, 12, 4, 80),
            (12, 12, 4, 28, 8, 120),
            (12, 12, 6, 43, 8, 160),   # ~30% fill
            (16, 16, 8, 76, 16, 240),  # ~30% fill
            (16, 16, 8, 128, 16, 320), # 50% fill
        ]

    for (h, w, r, s, t, k) in scramble_grid:
        for seed in range(args.seeds):
            ins, witness = scramble_with_witness(
                h, w, n_robots=r, n_shelves=s, n_targets=t, k=k, seed=seed
            )
            ok, errs, _ = validate_plan(ins, witness)
            if not ok:
                raise RuntimeError(f"witness invalid for {ins.name}: {errs}")
            save_instance(ins, out / "scramble" / f"{ins.name}.yaml")
            count += 1

    # ---- ddmapd family: density sweep, perimeter agents ----
    if args.small:
        ddmapd_grid = [
            (8, 8, 2, 0.2, 2),
            (10, 10, 3, 0.3, 3),
        ]
    else:
        ddmapd_grid = [
            (10, 10, 3, 0.3, 3),
            (16, 16, 4, 0.3, 8),
            (16, 16, 8, 0.5, 8),
            (24, 24, 8, 0.4, 16),
            (24, 24, 12, 0.6, 16),
        ]

    for (h, w, r, d, t) in ddmapd_grid:
        for seed in range(args.seeds):
            ins = ddmapd_instance(
                h, w, n_robots=r, block_density=d, n_targets=t, seed=seed
            )
            save_instance(ins, out / "ddmapd" / f"{ins.name}.yaml")
            count += 1

    print(f"generated {count} instances under {out}/")


if __name__ == "__main__":
    main()
