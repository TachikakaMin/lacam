#!/usr/bin/env python3
"""Generate a warehouse-block DD testcase and a matching HTML preview.

The warehouse geometry is a repeated pattern of:

    aisle (width A) + storage block (size B)

in both dimensions.  Shelves, target starts, and target goals are sampled
only from storage cells.  Every block receives exactly the same number of
shelves, so density is uniform within one testcase.

The current core DD YAML format has no storage/drop-zone mask.  Consequently
this generator enforces aisle exclusion for the generated start/goal layout,
but the existing planner/validator does not yet forbid an intermediate DROP
on an aisle cell.
"""

import argparse
import json
import math
import random
import sys
from pathlib import Path
from typing import Dict, List, Set

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ddbench.instance import Cell, load_instance


def block_geometry(
    height: int,
    width: int,
    block_size: int,
    aisle_width: int,
) -> List[List[Cell]]:
    period = block_size + aisle_width
    if height % period or width % period:
        raise ValueError(
            "height and width must be divisible by block_size + aisle_width "
            f"(got {height}x{width}, period={period})"
        )

    blocks: List[List[Cell]] = []
    for base_r in range(0, height, period):
        for base_c in range(0, width, period):
            r0 = base_r + aisle_width
            c0 = base_c + aisle_width
            blocks.append(
                [
                    (r, c)
                    for r in range(r0, r0 + block_size)
                    for c in range(c0, c0 + block_size)
                ]
            )
    return blocks


def choose_robots(
    height: int,
    width: int,
    storage: Set[Cell],
    n_robots: int,
    rng: random.Random,
) -> List[Cell]:
    corridors = [
        (r, c)
        for r in range(height)
        for c in range(width)
        if (r, c) not in storage
    ]
    if n_robots > len(corridors):
        raise ValueError("more robots requested than corridor cells")

    # Prefer aisle intersections because they make the preview easy to read.
    corridor_set = set(corridors)
    intersections = [
        p
        for p in corridors
        if any(
            (p[0] + dr, p[1]) in corridor_set
            for dr in (-1, 1)
            if 0 <= p[0] + dr < height
        )
        and any(
            (p[0], p[1] + dc) in corridor_set
            for dc in (-1, 1)
            if 0 <= p[1] + dc < width
        )
    ]
    if len(intersections) >= n_robots:
        return sorted(rng.sample(intersections, n_robots))

    selected = list(intersections)
    remaining = [p for p in corridors if p not in set(selected)]
    selected.extend(rng.sample(remaining, n_robots - len(selected)))
    return sorted(selected)


def build_case(
    height: int,
    width: int,
    block_size: int,
    aisle_width: int,
    density: float,
    n_robots: int,
    n_targets: int,
    seed: int,
) -> Dict:
    if not 0.0 < density < 1.0:
        raise ValueError("density must be strictly between 0 and 1")

    rng = random.Random(seed)
    blocks = block_geometry(height, width, block_size, aisle_width)
    storage = {p for block in blocks for p in block}
    corridor = {
        (r, c)
        for r in range(height)
        for c in range(width)
        if (r, c) not in storage
    }

    cells_per_block = block_size * block_size
    # Round halves upward, rather than Python's banker rounding, so a
    # requested 50% density in a 3x3 block becomes 5/9 instead of 4/9.
    shelves_per_block = int(math.floor(cells_per_block * density + 0.5))
    if not 0 < shelves_per_block < cells_per_block:
        raise ValueError(
            "density must leave at least one shelf and one vacancy per block"
        )
    actual_density = shelves_per_block / cells_per_block

    occupied_by_block: List[List[Cell]] = []
    vacant_by_block: List[List[Cell]] = []
    for block in blocks:
        occupied = sorted(rng.sample(block, shelves_per_block))
        occupied_set = set(occupied)
        occupied_by_block.append(occupied)
        vacant_by_block.append(sorted(p for p in block if p not in occupied_set))

    shelves = sorted(p for block in occupied_by_block for p in block)
    vacancies = sorted(p for block in vacant_by_block for p in block)
    if n_targets > len(shelves) or n_targets > len(vacancies):
        raise ValueError("not enough shelves or storage vacancies for targets")

    # Balance targets across blocks.  When there is more than one block,
    # every target is sent to a vacancy in a different block.
    if len(blocks) > 1:
        max_balanced_targets = len(blocks) * min(
            shelves_per_block, cells_per_block - shelves_per_block
        )
        if n_targets > max_balanced_targets:
            raise ValueError(
                "target count is too high to distribute evenly across blocks"
            )
        source_blocks: List[int] = []
        while len(source_blocks) < n_targets:
            cycle = list(range(len(blocks)))
            rng.shuffle(cycle)
            source_blocks.extend(cycle)
        source_blocks = source_blocks[:n_targets]

        shuffled_occupied = [list(cells) for cells in occupied_by_block]
        shuffled_vacant = [list(cells) for cells in vacant_by_block]
        for cells in shuffled_occupied:
            rng.shuffle(cells)
        for cells in shuffled_vacant:
            rng.shuffle(cells)

        target_starts = [
            shuffled_occupied[block_i].pop() for block_i in source_blocks
        ]
        destination_blocks = [
            (block_i + 1) % len(blocks) for block_i in source_blocks
        ]
        target_goals = [
            shuffled_vacant[block_i].pop()
            for block_i in destination_blocks
        ]
    else:
        target_starts = rng.sample(shelves, n_targets)
        target_goals = rng.sample(vacancies, n_targets)

    robots = choose_robots(height, width, storage, n_robots, rng)
    name = (
        f"warehouse_blocks_h{height}w{width}_b{block_size}_a{aisle_width}_"
        f"d{int(round(density * 100)):02d}_r{n_robots}_t{n_targets}_seed{seed}"
    )
    targets = [
        {"id": f"b{i}", "start": target_starts[i], "goal": target_goals[i]}
        for i in range(n_targets)
    ]

    case = {
        "name": name,
        "height": height,
        "width": width,
        "block_size": block_size,
        "aisle_width": aisle_width,
        "requested_density": density,
        "density": actual_density,
        "shelves_per_block": shelves_per_block,
        "blocks": blocks,
        "storage": storage,
        "corridor": corridor,
        "robots": robots,
        "shelves": shelves,
        "targets": targets,
    }
    validate_warehouse_contract(case)
    return case


def validate_warehouse_contract(case: Dict) -> None:
    storage = case["storage"]
    corridor = case["corridor"]
    shelves = set(case["shelves"])
    targets = case["targets"]

    assert storage.isdisjoint(corridor)
    assert len(storage | corridor) == case["height"] * case["width"]
    assert shelves <= storage
    assert set(case["robots"]) <= corridor
    assert all(tuple(t["start"]) in storage for t in targets)
    assert all(tuple(t["goal"]) in storage for t in targets)
    assert all(tuple(t["goal"]) not in shelves for t in targets)
    assert all(tuple(t["start"]) in shelves for t in targets)

    expected = case["shelves_per_block"]
    for i, block in enumerate(case["blocks"]):
        actual = len(set(block) & shelves)
        assert actual == expected, (
            f"block {i} has {actual} shelves, expected {expected}"
        )


def write_yaml(case: Dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    h = case["height"]
    w = case["width"]
    requested = case["requested_density"]
    actual = case["density"]
    lines = [
        "# Warehouse-block testcase generated by generate_warehouse_block_sample.py",
        f"# geometry: {h}x{w}, block={case['block_size']}x{case['block_size']}, "
        f"aisle_width={case['aisle_width']}",
        f"# requested density: {requested:.4f}; actual uniform density: {actual:.4f} "
        f"({case['shelves_per_block']} shelves per block)",
        "# contract: shelves, target starts, and target goals are storage cells;",
        "#           robots start in corridors; no initial shelf is in a corridor.",
        "# NOTE: corridor exclusion applies to generated placements and goals.",
        "#       The current DD action model still permits intermediate DROP there.",
        f"name: {case['name']}",
        "map: |",
    ]
    lines.extend(f"  {'.' * w}" for _ in range(h))
    lines.extend(
        [
            "warehouse_layout:",
            f"  block_size: {case['block_size']}",
            f"  aisle_width: {case['aisle_width']}",
            f"  requested_density: {requested:.8f}",
            f"  actual_density: {actual:.8f}",
            f"  shelves_per_block: {case['shelves_per_block']}",
            "  corridor_policy: initial_and_goal_placements_excluded",
            "  block_origins:",
        ]
    )
    lines.extend(
        f"    - [{block[0][0]}, {block[0][1]}]"
        for block in case["blocks"]
    )
    lines.append("robots:")
    lines.extend(f"  - [{r}, {c}]" for r, c in case["robots"])
    lines.append("shelves:")
    lines.extend(f"  - [{r}, {c}]" for r, c in case["shelves"])
    lines.append("targets:")
    for target in case["targets"]:
        sr, sc = target["start"]
        gr, gc = target["goal"]
        lines.extend(
            [
                f"  - id: {target['id']}",
                f"    start: [{sr}, {sc}]",
                f"    goal: [{gr}, {gc}]",
            ]
        )
    lines.extend(
        [
            "flags:",
            "  remove_on_complete: false",
            "  robots_return_to_rest: false",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<style>
  :root {
    color-scheme: dark;
    --bg: #111827;
    --panel: #1f2937;
    --muted: #9ca3af;
    --line: #4b5563;
    --aisle: #26384a;
    --storage: #d9c7a2;
    --shelf: #7c4a21;
    --target: #f59e0b;
    --goal: #22c55e;
    --robot: #3b82f6;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0;
    padding: 24px;
    background: var(--bg);
    color: #f9fafb;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
  }
  h1 { margin: 0 0 8px; font-size: 24px; }
  .subtitle { margin: 0 0 20px; color: var(--muted); }
  .layout {
    display: grid;
    grid-template-columns: minmax(500px, 720px) minmax(280px, 390px);
    gap: 22px;
    align-items: start;
  }
  .canvas-wrap, .panel {
    background: var(--panel);
    border: 1px solid #374151;
    border-radius: 12px;
    padding: 16px;
    box-shadow: 0 12px 35px rgba(0, 0, 0, .22);
  }
  canvas {
    display: block;
    width: min(100%, 680px);
    height: auto;
    background: #0f172a;
    border-radius: 7px;
  }
  #hover { min-height: 24px; margin-top: 10px; color: #cbd5e1; font-size: 14px; }
  .stats {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
    margin-bottom: 18px;
  }
  .stat { background: #111827; border-radius: 8px; padding: 10px 12px; }
  .stat b { display: block; font-size: 20px; margin-top: 2px; }
  .stat span { color: var(--muted); font-size: 12px; }
  h2 { margin: 18px 0 10px; font-size: 16px; }
  .legend { display: grid; grid-template-columns: 1fr 1fr; gap: 9px 12px; }
  .legend-item { display: flex; align-items: center; gap: 8px; font-size: 13px; }
  .swatch { width: 18px; height: 18px; border-radius: 3px; flex: 0 0 auto; }
  .note {
    margin-top: 18px;
    padding: 12px;
    border-left: 3px solid #f59e0b;
    background: #2b2418;
    color: #fde68a;
    font-size: 13px;
    line-height: 1.55;
  }
  code { color: #bfdbfe; }
  @media (max-width: 980px) {
    .layout { grid-template-columns: 1fr; }
  }
</style>
</head>
<body>
<h1>__MAP_SIZE__ Amazon 风格 warehouse-block testcase</h1>
<p class="subtitle"><code>__TITLE__</code> · 鼠标悬停格子可查看坐标和类型</p>
<div class="layout">
  <div class="canvas-wrap">
    <canvas id="map"></canvas>
    <div id="hover">将鼠标移到地图上查看单元格。</div>
  </div>
  <aside class="panel">
    <div class="stats">
      <div class="stat"><span>地图</span><b id="mapSize"></b></div>
      <div class="stat"><span>Block</span><b id="blockSize"></b></div>
      <div class="stat"><span>Block 数量</span><b id="blockCount"></b></div>
      <div class="stat"><span>通道宽度</span><b id="aisleWidth"></b></div>
      <div class="stat"><span>每块货架密度</span><b id="density"></b></div>
      <div class="stat"><span>货架 / 机器人 / 任务</span><b id="counts"></b></div>
    </div>
    <h2>图例</h2>
    <div class="legend">
      <div class="legend-item"><span class="swatch" style="background:var(--aisle)"></span>1 格通道</div>
      <div class="legend-item"><span class="swatch" style="background:var(--storage)"></span>空 storage slot</div>
      <div class="legend-item"><span class="swatch" style="background:var(--shelf)"></span>匿名货架</div>
      <div class="legend-item"><span class="swatch" style="background:var(--target)"></span>目标货架</div>
      <div class="legend-item"><span class="swatch" style="border:3px solid var(--goal)"></span>目标空位</div>
      <div class="legend-item"><span class="swatch" style="background:var(--robot);border-radius:50%"></span>机器人</div>
    </div>
    <div class="note">
      生成器已验证：每个 block 的尺寸和货架数完全相同，所有初始货架、目标起点和目标终点都位于 storage block，通道中没有初始货架。这里的通道禁放约束针对 testcase 的初始与目标布局；现有求解动作语义仍允许中间 DROP。
    </div>
  </aside>
</div>
<script>
const D = __DATA__;
const cv = document.getElementById("map");
const ctx = cv.getContext("2d");
const cell = 32;
const dpr = window.devicePixelRatio || 1;
cv.width = D.width * cell * dpr;
cv.height = D.height * cell * dpr;
cv.style.aspectRatio = `${D.width} / ${D.height}`;
ctx.scale(dpr, dpr);

const key = p => `${p[0]},${p[1]}`;
const storage = new Set(D.storage.map(key));
const shelves = new Set(D.shelves.map(key));
const targetAt = new Map(D.targets.map(t => [key(t.start), t.id]));
const goalAt = new Map(D.targets.map(t => [key(t.goal), t.id]));
const robotAt = new Map(D.robots.map((p, i) => [key(p), i]));

function draw() {
  ctx.clearRect(0, 0, D.width * cell, D.height * cell);
  for (let r = 0; r < D.height; r++) {
    for (let c = 0; c < D.width; c++) {
      const k = `${r},${c}`;
      ctx.fillStyle = storage.has(k) ? "#d9c7a2" : "#26384a";
      ctx.fillRect(c * cell, r * cell, cell, cell);
    }
  }

  ctx.strokeStyle = "rgba(255,255,255,.55)";
  ctx.lineWidth = 2;
  for (const block of D.blockOrigins) {
    ctx.strokeRect(
      block[1] * cell + 1,
      block[0] * cell + 1,
      D.blockSize * cell - 2,
      D.blockSize * cell - 2
    );
  }

  for (const p of D.shelves) {
    const k = key(p);
    const pad = 5;
    ctx.fillStyle = targetAt.has(k) ? "#f59e0b" : "#7c4a21";
    ctx.fillRect(p[1] * cell + pad, p[0] * cell + pad, cell - 2 * pad, cell - 2 * pad);
    if (targetAt.has(k)) {
      ctx.fillStyle = "#111827";
      ctx.font = "bold 9px sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(targetAt.get(k), p[1] * cell + cell / 2, p[0] * cell + cell / 2);
    }
  }

  ctx.lineWidth = 3;
  ctx.strokeStyle = "#22c55e";
  for (const target of D.targets) {
    const p = target.goal;
    ctx.strokeRect(p[1] * cell + 4, p[0] * cell + 4, cell - 8, cell - 8);
    ctx.fillStyle = "#14532d";
    ctx.font = "bold 8px sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(target.id, p[1] * cell + cell / 2, p[0] * cell + cell / 2);
  }

  for (let i = 0; i < D.robots.length; i++) {
    const p = D.robots[i];
    ctx.beginPath();
    ctx.fillStyle = "#3b82f6";
    ctx.strokeStyle = "#dbeafe";
    ctx.lineWidth = 1.5;
    ctx.arc(p[1] * cell + cell / 2, p[0] * cell + cell / 2, cell * .31, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "white";
    ctx.font = "bold 9px sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(`R${i}`, p[1] * cell + cell / 2, p[0] * cell + cell / 2);
  }

  ctx.strokeStyle = "rgba(17,24,39,.32)";
  ctx.lineWidth = 1;
  for (let r = 0; r <= D.height; r++) {
    ctx.beginPath();
    ctx.moveTo(0, r * cell);
    ctx.lineTo(D.width * cell, r * cell);
    ctx.stroke();
  }
  for (let c = 0; c <= D.width; c++) {
    ctx.beginPath();
    ctx.moveTo(c * cell, 0);
    ctx.lineTo(c * cell, D.height * cell);
    ctx.stroke();
  }
}

document.getElementById("mapSize").textContent = `${D.height}×${D.width}`;
document.getElementById("blockSize").textContent = `${D.blockSize}×${D.blockSize}`;
document.getElementById("blockCount").textContent = D.blockOrigins.length;
document.getElementById("aisleWidth").textContent = D.aisleWidth;
document.getElementById("density").textContent =
  `${(D.density * 100).toFixed(1)}% (${D.shelvesPerBlock}/${D.blockSize * D.blockSize})`;
document.getElementById("counts").textContent =
  `${D.shelves.length} / ${D.robots.length} / ${D.targets.length}`;

cv.addEventListener("mousemove", event => {
  const rect = cv.getBoundingClientRect();
  const c = Math.floor((event.clientX - rect.left) * D.width / rect.width);
  const r = Math.floor((event.clientY - rect.top) * D.height / rect.height);
  const k = `${r},${c}`;
  const items = [storage.has(k) ? "storage slot" : "corridor"];
  if (shelves.has(k)) items.push(targetAt.has(k) ? `target shelf ${targetAt.get(k)}` : "anonymous shelf");
  if (goalAt.has(k)) items.push(`goal ${goalAt.get(k)}`);
  if (robotAt.has(k)) items.push(`robot R${robotAt.get(k)}`);
  document.getElementById("hover").textContent = `(${r}, ${c}) · ${items.join(" · ")}`;
});
cv.addEventListener("mouseleave", () => {
  document.getElementById("hover").textContent = "将鼠标移到地图上查看单元格。";
});
draw();
</script>
</body>
</html>
"""


def write_html(case: Dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "name": case["name"],
        "height": case["height"],
        "width": case["width"],
        "blockSize": case["block_size"],
        "aisleWidth": case["aisle_width"],
        "requestedDensity": case["requested_density"],
        "density": case["density"],
        "shelvesPerBlock": case["shelves_per_block"],
        "blockOrigins": [block[0] for block in case["blocks"]],
        "storage": sorted(case["storage"]),
        "shelves": case["shelves"],
        "robots": case["robots"],
        "targets": case["targets"],
    }
    html = HTML_TEMPLATE.replace("__TITLE__", case["name"])
    html = html.replace(
        "__MAP_SIZE__", f"{case['height']}×{case['width']}"
    )
    html = html.replace("__DATA__", json.dumps(data, separators=(",", ":")))
    path.write_text(html, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--height", type=int, default=20)
    parser.add_argument("--width", type=int, default=20)
    parser.add_argument("--block-size", type=int, default=4)
    parser.add_argument("--aisle-width", type=int, default=1)
    parser.add_argument("--density", type=float, default=0.75)
    parser.add_argument("--robots", type=int, default=8)
    parser.add_argument("--targets", type=int, default=12)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--instance-dir",
        type=Path,
        default=here / "instances_warehouse_blocks_preview",
    )
    parser.add_argument(
        "--html",
        type=Path,
        default=here / "viz_web" / "warehouse_blocks_20x20_sample.html",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    case = build_case(
        height=args.height,
        width=args.width,
        block_size=args.block_size,
        aisle_width=args.aisle_width,
        density=args.density,
        n_robots=args.robots,
        n_targets=args.targets,
        seed=args.seed,
    )
    instance_path = args.instance_dir / f"{case['name']}.yaml"
    write_yaml(case, instance_path)
    write_html(case, args.html)

    loaded = load_instance(instance_path)
    errors = loaded.validate_static()
    if errors:
        raise ValueError(f"generated DD instance failed validation: {errors}")

    print(f"instance={instance_path}")
    print(f"visualization={args.html}")
    print(
        "summary="
        f"{case['height']}x{case['width']}, "
        f"{len(case['blocks'])} blocks, "
        f"{case['block_size']}x{case['block_size']} each, "
        f"aisle={case['aisle_width']}, "
        f"density={case['density']:.1%}, "
        f"shelves={len(case['shelves'])}, "
        f"robots={len(case['robots'])}, "
        f"targets={len(case['targets'])}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
