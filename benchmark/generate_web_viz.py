#!/usr/bin/env python3
"""Generate a self-contained interactive HTML visualization of a
double-deck plan (debug.md task 12, web edition).

The plan is validated through the authoritative Python validator first;
the HTML embeds only the instance + op sequence and replays it in JS.

Usage:
  python3 generate_web_viz.py INSTANCE.yaml PLAN.plan OUT.html [--title T]
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from ddbench.instance import load_instance
from ddbench.validator import apply_joint_action, initial_state, is_goal

TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>__TITLE__</title>
<style>
  body { font-family: -apple-system, "Segoe UI", sans-serif; margin: 16px;
         background: #1e1e24; color: #eee; }
  h2 { margin: 4px 0 10px; font-weight: 600; }
  #bar { display: flex; gap: 10px; align-items: center; flex-wrap: wrap;
         margin-bottom: 10px; }
  button { background: #3b82f6; color: #fff; border: 0; padding: 6px 14px;
           border-radius: 6px; cursor: pointer; font-size: 14px; }
  button:hover { background: #2563eb; }
  input[type=range] { width: 320px; }
  #stats { font-size: 13px; color: #bbb; }
  canvas { background: #2a2a32; border-radius: 8px; display: block; }
  .legend { font-size: 12px; color: #aaa; margin-top: 8px; }
  .sw { display: inline-block; width: 10px; height: 10px; border-radius: 2px;
        margin: 0 4px 0 10px; vertical-align: middle; }
</style>
</head>
<body>
<h2>__TITLE__</h2>
<div id="bar">
  <button id="play">▶ 播放</button>
  <button id="stepb">−1</button>
  <button id="stepf">+1</button>
  <label>速度 <input id="speed" type="range" min="1" max="60" value="15"></label>
  <label>进度 <input id="seek" type="range" min="0" value="0" style="width:360px"></label>
  <span id="stats"></span>
</div>
<canvas id="cv"></canvas>
<div class="legend">
  <span class="sw" style="background:#555"></span>墙
  <span class="sw" style="background:#8b5a2b"></span>匿名货架
  <span class="sw" style="background:#f59e0b"></span>目标货架
  <span class="sw" style="border:2px solid #22c55e"></span>goal（多个绿框=任一即可）
  <span class="sw" style="background:#3b82f6;border-radius:50%"></span>空载机器人
  <span class="sw" style="background:#ef4444;border-radius:50%"></span>载货机器人
  <span class="sw" style="border:2px solid #fff;background:#8b5a2b"></span>被举起的货架（白边，随机器人移动）
  <span class="sw" style="background:#16a34a"></span>已完成的目标货架
</div>
<script>
const D = __DATA__;
const H = D.h, W = D.w;
const gsets = D.targets.map(x => new Set(x.gs.map(p => p[0] * W + p[1])));
const goalUnion = new Set();
for (const gs of gsets) for (const k of gs) goalUnion.add(k);
const cell = Math.max(6, Math.min(28, Math.floor(1200 / W)));
const cv = document.getElementById("cv");
cv.width = W * cell; cv.height = H * cell;
const cx = cv.getContext("2d");

// mutable state
let t = 0;
let robots, kappa, tpos, anon;
function reset() {
  robots = D.robots.map(p => [...p]);
  kappa = D.robots.map(_ => -1);          // -1 free, -2 anon, >=0 target idx
  tpos = D.targets.map(x => [...x.s]);
  anon = new Set(D.anon.map(p => p[0] * W + p[1]));
}
function key(p) { return p[0] * W + p[1]; }
function applyStep(ops) {
  // lifts read the pre-step grounded state
  const tgtAt = new Map();
  const carried = new Set(kappa.filter(k => k >= 0));
  D.targets.forEach((_, b) => { if (!carried.has(b)) tgtAt.set(key(tpos[b]), b); });
  for (let i = 0; i < ops.length; i++) {
    const op = ops[i];
    if (op[0] === "m") {
      robots[i] = [op[1], op[2]];
      if (kappa[i] >= 0) tpos[kappa[i]] = [op[1], op[2]];
    } else if (op[0] === "l") {
      const k = key(robots[i]);
      if (tgtAt.has(k)) kappa[i] = tgtAt.get(k);
      else { kappa[i] = -2; anon.delete(k); }
    } else if (op[0] === "d") {
      if (kappa[i] === -2) anon.add(key(robots[i]));
      kappa[i] = -1;
    }
  }
}
function goto_(target) {
  if (target < t) { reset(); t = 0; }
  while (t < target) { applyStep(D.plan[t]); t++; }
  draw();
}
function drawCellRect(r, c, color, pad) {
  cx.fillStyle = color;
  cx.fillRect(c * cell + pad, r * cell + pad, cell - 2 * pad, cell - 2 * pad);
}
function draw() {
  cx.clearRect(0, 0, cv.width, cv.height);
  // grid + walls
  cx.strokeStyle = "#3a3a44"; cx.lineWidth = 1;
  for (let r = 0; r < H; r++) for (let c = 0; c < W; c++) {
    cx.strokeRect(c * cell, r * cell, cell, cell);
    if (D.walls[r][c]) drawCellRect(r, c, "#555", 0);
  }
  // goals: union of the eligible sets (a goal-POOL instance outlines
  // every eligible cell — any of them is a valid exit)
  cx.lineWidth = Math.max(1.5, cell * 0.09);
  cx.strokeStyle = "#22c55e";
  for (const k of goalUnion)
    cx.strokeRect((k % W) * cell + 1.5, Math.floor(k / W) * cell + 1.5,
                  cell - 3, cell - 3);
  // anon shelves
  for (const k of anon) drawCellRect(Math.floor(k / W), k % W, "#8b5a2b", cell * 0.18);
  // target shelves (green once grounded on ANY eligible goal cell)
  const carried = new Set(kappa.filter(k => k >= 0));
  D.targets.forEach((g, b) => {
    const done = !carried.has(b) && gsets[b].has(key(tpos[b]));
    drawCellRect(tpos[b][0], tpos[b][1], done ? "#16a34a" : "#f59e0b", cell * 0.18);
  });
  // carried shelves ride ON their robot: anon carried shelves are not in
  // `anon` while airborne — draw them at the carrier cell, plus a white
  // "lifted" outline for every carried shelf (target or anon)
  for (let i = 0; i < robots.length; i++) {
    if (kappa[i] === -2)
      drawCellRect(robots[i][0], robots[i][1], "#8b5a2b", cell * 0.18);
    if (kappa[i] !== -1) {
      cx.strokeStyle = "#ffffff";
      cx.lineWidth = Math.max(1, cell * 0.06);
      cx.strokeRect(robots[i][1] * cell + cell * 0.18,
                    robots[i][0] * cell + cell * 0.18,
                    cell * 0.64, cell * 0.64);
    }
  }
  // robots
  for (let i = 0; i < robots.length; i++) {
    cx.fillStyle = kappa[i] === -1 ? "#3b82f6" : "#ef4444";
    cx.beginPath();
    cx.arc(robots[i][1] * cell + cell / 2, robots[i][0] * cell + cell * 0.68,
           Math.max(2.2, cell * 0.16), 0, Math.PI * 2);
    cx.fill();
  }
  // stats
  let done = 0;
  D.targets.forEach((g, b) => {
    if (!carried.has(b) && gsets[b].has(key(tpos[b]))) done++;
  });
  document.getElementById("stats").textContent =
    `t=${t}/${D.plan.length}  完成 ${done}/${D.targets.length}  载运中 ${carried.size}`;
  document.getElementById("seek").value = t;
}
reset(); draw();
document.getElementById("seek").max = D.plan.length;
let timer = null;
const playBtn = document.getElementById("play");
function tick() {
  if (t >= D.plan.length) { stop_(); return; }
  goto_(t + 1);
}
function stop_() { clearInterval(timer); timer = null; playBtn.textContent = "▶ 播放"; }
playBtn.onclick = () => {
  if (timer) { stop_(); return; }
  if (t >= D.plan.length) goto_(0);
  playBtn.textContent = "⏸ 暂停";
  const fps = () => +document.getElementById("speed").value;
  timer = setInterval(tick, 1000 / fps());
  document.getElementById("speed").oninput = () => {
    if (timer) { clearInterval(timer); timer = setInterval(tick, 1000 / fps()); }
  };
};
document.getElementById("stepf").onclick = () => goto_(Math.min(t + 1, D.plan.length));
document.getElementById("stepb").onclick = () => goto_(Math.max(t - 1, 0));
document.getElementById("seek").oninput = e => goto_(+e.target.value);
</script>
</body>
</html>
"""


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("instance")
    ap.add_argument("plan")
    ap.add_argument("out")
    ap.add_argument("--title", default=None)
    args = ap.parse_args()

    ins = load_instance(args.instance)
    plan = parse_plan(args.plan)

    # authoritative validation before embedding
    s = initial_state(ins)
    for joint in plan:
        s = apply_joint_action(ins, s, joint)
    assert is_goal(ins, s), "plan does not reach the goal"

    tset = {tuple(t.start) for t in ins.targets}
    data = {
        "h": ins.height,
        "w": ins.width,
        "walls": [[1 if ins.grid[r][c] else 0 for c in range(ins.width)]
                  for r in range(ins.height)],
        "robots": [list(q) for q in ins.robots],
        "anon": [list(p) for p in ins.shelves if tuple(p) not in tset],
        "targets": [{"id": t.id, "s": list(t.start),
                     "gs": [list(g) for g in t.eligible_goals()]}
                    for t in ins.targets],
        "plan": [
            [["w"] if a[0] == "wait" else
             (["m", a[1][0], a[1][1]] if a[0] == "move" else
              (["l"] if a[0] == "lift" else ["d"]))
             for a in joint]
            for joint in plan
        ],
    }
    title = args.title or (f"{ins.name} — {ins.height}x{ins.width}, "
                           f"{len(ins.robots)} robots, {len(ins.shelves)} "
                           f"shelves, {len(ins.targets)} targets, "
                           f"makespan {len(plan)}")
    html = TEMPLATE.replace("__TITLE__", title).replace(
        "__DATA__", json.dumps(data, separators=(",", ":")))
    Path(args.out).write_text(html)
    print(f"wrote {args.out} ({Path(args.out).stat().st_size // 1024} KB, "
          f"{len(plan)} steps)")


if __name__ == "__main__":
    main()
