#!/usr/bin/env python3
"""Render an LDraw model using geometry from the official complete.zip.

This is a verification renderer, not a replacement for an LDraw editor.  It
expands type-1 references and renders type-3/4 faces from the official parts
library so incorrect transforms are visible before the model is published.
"""
from __future__ import annotations

import os
import sys
import zipfile
from functools import lru_cache
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

BASE = Path(__file__).resolve().parent
ZIP = Path(os.environ.get("LDRAW_ZIP", "/tmp/ldraw-complete.zip"))
MODEL = Path(sys.argv[1]) if len(sys.argv) > 1 else BASE / "brougham_exact_model.ldr"
OUT = MODEL.with_suffix(".png")

COLORS = {
    0: "#596068", 15: "#f1f1ec", 47: "#d9f4ff",
    70: "#6c3a20", 46: "#ffd92f", 320: "#720e18",
    4: "#c91a09", 6: "#a65318", 7: "#9ba19d", 16: "#777777",
}

zf = zipfile.ZipFile(ZIP)
zip_names = {n.lower(): n for n in zf.namelist()}


def resolve(name):
    name = name.replace("\\", "/").lower()
    no_s = name[2:] if name.startswith("s/") else name
    no_48 = name[3:] if name.startswith("48/") else name
    no_8 = name[2:] if name.startswith("8/") else name
    candidates = [
        "ldraw/parts/" + name,
        "ldraw/p/" + name,
        "ldraw/parts/s/" + no_s,
        "ldraw/p/48/" + no_48,
        "ldraw/p/8/" + no_8,
    ]
    for c in candidates:
        if c in zip_names:
            return zip_names[c]
    raise FileNotFoundError(name)


def transform(points, matrix, offset):
    return points @ matrix.T + offset


@lru_cache(maxsize=None)
def raw_lines(name):
    return zf.read(resolve(name)).decode("latin-1").splitlines()


def expand(name, matrix, offset, inherited_color, faces, depth=0):
    if depth > 40:
        raise RuntimeError(f"reference depth exceeded at {name}")
    for line in raw_lines(name):
        fields = line.strip().split()
        if not fields:
            continue
        typ = fields[0]
        if typ == "1" and len(fields) >= 15:
            color = int(fields[1])
            child_color = inherited_color if color == 16 else color
            off = np.array(list(map(float, fields[2:5])))
            mat = np.array(list(map(float, fields[5:14]))).reshape(3, 3)
            child_name = " ".join(fields[14:])
            expand(child_name, matrix @ mat, matrix @ off + offset,
                   child_color, faces, depth + 1)
        elif typ in ("3", "4"):
            color = int(fields[1])
            face_color = inherited_color if color in (16, 24) else color
            count = 3 if typ == "3" else 4
            pts = np.array(list(map(float, fields[2:2 + count * 3]))).reshape(count, 3)
            faces.append((transform(pts, matrix, offset), face_color))


def parse_model(path):
    faces = []
    max_step = int(os.environ.get("LDRAW_MAX_STEP", "999"))
    current_step = 0
    for line in path.read_text().splitlines():
        fields = line.strip().split()
        if len(fields) >= 5 and fields[:3] == ["0", "STEP", "//"] and \
                fields[3] == "PDF":
            current_step = int(fields[4])
            continue
        if not fields or fields[0] != "1":
            continue
        if current_step > max_step:
            continue
        color = int(fields[1])
        off = np.array(list(map(float, fields[2:5])))
        mat = np.array(list(map(float, fields[5:14]))).reshape(3, 3)
        name = " ".join(fields[14:])
        expand(name, mat, off, color, faces)
    return faces


def main():
    ldraw_faces = parse_model(MODEL)
    # Matplotlib uses Z as the vertical axis, while LDraw uses downward Y.
    # Convert every vertex, not just the camera bounds:
    #   (LDraw X, Y, Z) -> (view X, forward Z, up -Y).
    faces = []
    for pts, color in ldraw_faces:
        view_pts = pts[:, [0, 2, 1]].copy()
        view_pts[:, 2] *= -1
        faces.append((view_pts, color))
    verts = np.concatenate([f[0] for f in faces])
    lo, hi = verts.min(axis=0), verts.max(axis=0)
    center = (lo + hi) / 2
    span = max(hi - lo)
    fig = plt.figure(figsize=(16, 11), facecolor="#eef2f6")
    groups = {}
    for pts, color in faces:
        groups.setdefault(color, []).append(pts)

    ax = fig.add_subplot(2, 2, 1, projection="3d")
    for color, polys in groups.items():
        rgba = matplotlib.colors.to_rgba(COLORS.get(color, "#888888"))
        alpha = .28 if color == 47 else 1.0
        coll = Poly3DCollection(
            polys, facecolors=[(*rgba[:3], alpha)],
            edgecolors=(0.05, 0.05, 0.05, .20), linewidths=.08)
        ax.add_collection3d(coll)
    ax.set_xlim(center[0] - span / 2, center[0] + span / 2)
    ax.set_ylim(center[1] - span / 2, center[1] + span / 2)
    ax.set_zlim(center[2] - span / 2, center[2] + span / 2)
    azim = float(os.environ.get("LDRAW_AZIM", "-55"))
    ax.view_init(elev=22, azim=azim)
    ax.set_box_aspect((1, 1, 1))
    ax.set_axis_off()
    ax.set_title(f"perspective azim={azim:g}")

    def projection(index, dims, title):
        pax = fig.add_subplot(2, 2, index)
        for color, polys in groups.items():
            rgba = matplotlib.colors.to_rgba(COLORS.get(color, "#888888"))
            alpha = .28 if color == 47 else 1.0
            projected = [poly[:, dims] for poly in polys]
            coll = PolyCollection(
                projected, facecolors=[(*rgba[:3], alpha)],
                edgecolors=(0.05, 0.05, 0.05, .18), linewidths=.08)
            pax.add_collection(coll)
        all_2d = verts[:, dims]
        plo, phi = all_2d.min(axis=0), all_2d.max(axis=0)
        margin = max(phi - plo) * .06
        pax.set_xlim(plo[0] - margin, phi[0] + margin)
        pax.set_ylim(plo[1] - margin, phi[1] + margin)
        pax.set_aspect("equal", adjustable="box")
        pax.set_axis_off()
        pax.set_title(title)

    # View coordinates are (width X, forward Z, up -Y).
    projection(2, (1, 2), "orthographic side")
    projection(3, (0, 2), "orthographic front")
    projection(4, (0, 1), "orthographic top")
    fig.suptitle(f"{MODEL.name} — official LDraw geometry", fontsize=13)
    fig.tight_layout()
    fig.savefig(OUT, dpi=130)
    print(f"rendered {len(faces)} faces -> {OUT}")


if __name__ == "__main__":
    main()
