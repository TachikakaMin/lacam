#!/usr/bin/env python3
"""Double-deck schedule visualizer (design.md M0 exit item; debug.md P3).

Replays a dd_benchmark plan on its instance through the AUTHORITATIVE
validator and renders frames:

  ascii  — print every k-th configuration to stdout (headless-friendly)
  png    — one PNG per k-th step via matplotlib (if available)

Legend (ascii): '.'=free  '#'=anon shelf  'A'-'Z'=target shelf (id mod 26)
  lower deck robots are shown as 'r' when the cell has no shelf, '+' when a
  free robot stands under a shelf, '^' when carrying.

Usage:
  python3 visualize_dd_schedule.py INSTANCE.yaml PLAN.plan \
      [--mode ascii|png] [--every 5] [--out-dir frames/]
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from ddbench.instance import load_instance
from ddbench.validator import ANON, apply_joint_action, initial_state


def parse_plan(path):
    plan = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        joint = []
        for tok in line.split(";"):
            parts = tok.split()
            if parts[0] == "w":
                joint.append(("wait",))
            elif parts[0] == "m":
                joint.append(("move", (int(parts[1]), int(parts[2]))))
            elif parts[0] == "l":
                joint.append(("lift",))
            elif parts[0] == "d":
                joint.append(("drop",))
        plan.append(joint)
    return plan


def frame_ascii(ins, s, t):
    tgt_at = {}
    for i, (tid, cell) in enumerate(sorted(s.target_pos)):
        tgt_at[cell] = chr(ord("A") + (i % 26))
    anon = set(s.anon_occ)
    robot_at = {q: i for i, q in enumerate(s.robots)}
    lines = [f"t={t}"]
    for r in range(ins.height):
        row = ""
        for c in range(ins.width):
            p = (r, c)
            if ins.is_wall(p):
                row += "@"
                continue
            has_shelf = p in anon or p in tgt_at
            if p in robot_at:
                k = s.kappa[robot_at[p]]
                if k is not None:
                    row += "^"  # carrier (its shelf is at this cell)
                elif has_shelf:
                    row += "+"  # free robot under a shelf
                else:
                    row += "r"
            elif p in tgt_at:
                row += tgt_at[p]
            elif p in anon:
                row += "#"
            else:
                row += "."
        lines.append(row)
    return "\n".join(lines)


def frame_png(ins, s, t, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import patches

    fig, ax = plt.subplots(figsize=(ins.width / 3, ins.height / 3))
    ax.set_xlim(0, ins.width)
    ax.set_ylim(0, ins.height)
    ax.invert_yaxis()
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(f"t={t}")
    for r in range(ins.height):
        for c in range(ins.width):
            if ins.is_wall((r, c)):
                ax.add_patch(patches.Rectangle((c, r), 1, 1, color="black"))
    goals = {tuple(tt.goal) for tt in ins.targets}
    for (gr, gc) in goals:
        ax.add_patch(patches.Rectangle((gc, gr), 1, 1, facecolor="none",
                                       edgecolor="green", lw=1.5))
    tgt_cells = {cell for _, cell in s.target_pos}
    for cell in s.anon_occ:
        r, c = cell
        ax.add_patch(patches.Rectangle((c + 0.15, r + 0.15), 0.7, 0.7,
                                       color="saddlebrown", alpha=0.8))
    for _, cell in s.target_pos:
        r, c = cell
        ax.add_patch(patches.Rectangle((c + 0.15, r + 0.15), 0.7, 0.7,
                                       color="orange", alpha=0.9))
    for i, q in enumerate(s.robots):
        r, c = q
        color = "red" if s.kappa[i] is not None else "royalblue"
        ax.add_patch(patches.Circle((c + 0.5, r + 0.78), 0.16, color=color))
    fig.savefig(out_path, dpi=90, bbox_inches="tight")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("instance")
    ap.add_argument("plan")
    ap.add_argument("--mode", choices=["ascii", "png"], default="ascii")
    ap.add_argument("--every", type=int, default=5)
    ap.add_argument("--out-dir", default="frames")
    args = ap.parse_args()

    ins = load_instance(args.instance)
    plan = parse_plan(args.plan)
    s = initial_state(ins)
    frames = 0
    if args.mode == "png":
        Path(args.out_dir).mkdir(parents=True, exist_ok=True)
        frame_png(ins, s, 0, Path(args.out_dir) / "frame_00000.png")
    else:
        print(frame_ascii(ins, s, 0))
    for t, joint in enumerate(plan, start=1):
        s = apply_joint_action(ins, s, joint)  # validator-checked replay
        if t % args.every == 0 or t == len(plan):
            if args.mode == "png":
                frame_png(ins, s, t,
                          Path(args.out_dir) / f"frame_{t:05d}.png")
            else:
                print(frame_ascii(ins, s, t))
            frames += 1
    print(f"rendered {frames + 1} frames ({args.mode})", file=sys.stderr)


if __name__ == "__main__":
    main()
