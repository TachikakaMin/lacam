#!/usr/bin/env python3
"""Render the FINAL structure of a plan JSON as colored voxels from several
camera angles (headless). usage: render_final.py plan_x.json [...]"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def render(path: Path) -> Path:
    plan = json.loads(path.read_text())
    rows, cols = plan["rows"], plan["cols"]
    fh = plan.get("final_heights")
    if fh is None:
        # replay terrain to the end
        fh = [[0] * cols for _ in range(rows)]
        for ev in sorted(plan["terrain"], key=lambda e: e["t"]):
            fh[ev["r"]][ev["c"]] = ev["h"]
    colors = plan.get("colors", {})
    colors3 = plan.get("colors3", {})
    floating = plan.get("floating", {})
    maxh = max((max(r) for r in fh), default=1) or 1

    # crop to the structure bounding box (plus margin)
    rs = [r for r in range(rows) if any(fh[r])]
    cs = [c for c in range(cols) if any(fh[r][c] for r in range(rows))]
    r0, r1 = max(min(rs) - 1, 0), min(max(rs) + 2, rows)
    c0, c1 = max(min(cs) - 1, 0), min(max(cs) + 2, cols)
    R, C = r1 - r0, c1 - c0

    filled = np.zeros((C, R, maxh), dtype=bool)
    face = np.zeros((C, R, maxh, 4))
    default = np.array([0.91, 0.69, 0.29, 1.0])
    for r in range(r0, r1):
        for c in range(c0, c1):
            h = fh[r][c]
            base = colors.get(f"{r},{c}")
            z_lo = floating.get(f"{r},{c}", 1)  # floats: air below the slab
            for z in range(z_lo, h + 1):
                hexcol = colors3.get(f"{r},{c},{z}") or base
                rgba = (np.array(matplotlib.colors.to_rgba(hexcol))
                        if hexcol else default)
                filled[c - c0, r - r0, z - 1] = True
                face[c - c0, r - r0, z - 1] = rgba

    views = [(22, -60), (18, -120), (35, -90), (12, -20)]
    fig = plt.figure(figsize=(16, 10))
    for i, (elev, azim) in enumerate(views, 1):
        ax = fig.add_subplot(2, 2, i, projection="3d")
        ax.voxels(filled, facecolors=face, edgecolors=(0, 0, 0, 0.25),
                  linewidth=0.3)
        ax.set_box_aspect((C, R, maxh * 1.2))
        ax.view_init(elev=elev, azim=azim)
        ax.set_axis_off()
        ax.set_title(f"elev={elev} azim={azim}", fontsize=9)
    out = path.with_name(path.stem + "_3d.png")
    fig.tight_layout()
    fig.savefig(out, dpi=100, facecolor="#dfe7f0")
    plt.close(fig)
    return out


if __name__ == "__main__":
    for name in sys.argv[1:]:
        print("wrote", render(Path(name)))
