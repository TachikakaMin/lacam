#!/usr/bin/env python3
"""Generate paper-protocol instance suites (CREST arXiv:2603.28803 Table I).

  R2R-M  48x48, 460 shelves (20%), 230 rearranged (=0.1*48^2), N=32
  S2W-M  64x33, 384 shelves, all rearranged, N=32
  DnE-M  47x45, 672 shelves, ~403 rearranged, N=32

25 instances per layout, like the paper.  Large (-L) variants optional.
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from ddbench.generators import ddmapd_instance
from ddbench.instance import save_instance
from ddbench.paper_layouts import dne_instance, s2w_instance


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default="instances_paper")
    ap.add_argument("--seeds", type=int, default=25)
    ap.add_argument("--agents", type=int, default=32)
    ap.add_argument("--layouts", nargs="+", default=["r2r_m", "s2w_m", "dne_m"])
    args = ap.parse_args()
    out = Path(args.out_dir)

    t0 = time.time()
    count = 0
    for layout in args.layouts:
        d = out / layout
        d.mkdir(parents=True, exist_ok=True)
        for seed in range(args.seeds):
            if layout == "r2r_m":
                ins = ddmapd_instance(
                    48, 48, n_robots=args.agents, block_density=0.20,
                    n_targets=230, seed=seed,
                    name=f"r2rM_n32_seed{seed}",
                )
            elif layout == "s2w_m":
                ins = s2w_instance(
                    64, 33, n_robots=args.agents, n_shelves=384, seed=seed,
                    name=f"s2wM_n32_seed{seed}",
                )
            elif layout == "dne_m":
                ins = dne_instance(
                    47, 45, n_robots=args.agents, n_shelves=672, seed=seed,
                    n_rearranged=403,
                    name=f"dneM_n32_seed{seed}",
                )
            elif layout == "r2r_l":
                ins = ddmapd_instance(
                    96, 96, n_robots=args.agents, block_density=0.20,
                    n_targets=921, seed=seed,
                    name=f"r2rL_n{args.agents}_seed{seed}",
                )
            else:
                raise SystemExit(f"unknown layout {layout}")
            save_instance(ins, d / f"{ins.name}.yaml")
            count += 1
    print(f"generated {count} instances under {out}/ in {time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()
