#!/usr/bin/env python3
"""Render static PNG snapshots of demo plan JSONs (headless-friendly).

For each plan: left = final terrain heightmap with depot cells marked,
right = construction progress (placements/removals) over time.

usage: snapshot_plans.py plan_column.json [more ...]  -> plan_column.png
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def render(path: Path) -> Path:
    plan = json.loads(path.read_text())
    rows, cols, T = plan["rows"], plan["cols"], plan["T"]
    h = np.zeros((rows, cols), dtype=int)
    place_t, remove_t = [], []
    hh = np.zeros((rows, cols), dtype=int)
    for ev in sorted(plan["terrain"], key=lambda e: e["t"]):
        r, c, nh = ev["r"], ev["c"], ev["h"]
        (place_t if nh > hh[r][c] else remove_t).append(ev["t"])
        hh[r][c] = nh
    h = hh

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5),
                                   gridspec_kw={"width_ratios": [1.2, 1]})
    im = ax1.imshow(h, cmap="viridis", origin="upper")
    for r, c in plan["depot"]:
        ax1.plot(c, r, "s", mfc="none", mec="red", ms=6)
    for a in plan["agents"]:
        r, c = a["path"][-1]
        ax1.plot(c, r, "o", color="white", mec="black", ms=5)
    ax1.set_title(f"{path.stem}: final heights (red=depot, white=agents)")
    fig.colorbar(im, ax=ax1, shrink=0.8)

    tt = np.arange(T + 1)
    placed = np.searchsorted(np.sort(place_t), tt, side="right")
    removed = np.searchsorted(np.sort(remove_t), tt, side="right")
    ax2.plot(tt, placed, label=f"placements ({len(place_t)})")
    ax2.plot(tt, removed, label=f"removals ({len(remove_t)})")
    ax2.set_xlabel("t")
    ax2.set_title(f"progress, T={T}, agents={len(plan['agents'])}")
    ax2.legend()
    ax2.grid(alpha=0.3)

    out = path.with_suffix(".png")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out


if __name__ == "__main__":
    for name in sys.argv[1:] or ["plan_column.json"]:
        print("wrote", render(Path(name)))
