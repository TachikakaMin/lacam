#!/usr/bin/env python3
"""Generate the 20x20 warehouse-block testcase matrix and web dashboard."""

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ddbench.instance import load_instance
from generate_warehouse_block_sample import build_case, write_html, write_yaml


HEIGHT = 20
WIDTH = 20
AISLE_WIDTH = 1
BLOCK_SIZES: Tuple[int, ...] = (3, 4, 9)
DENSITY_LEVELS: Tuple[float, ...] = (0.25, 0.50, 0.75)
ROBOTS = 8
TARGETS = 12
SEED = 0


DASHBOARD_TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>20×20 Warehouse-block testcase suite</title>
<style>
  :root {
    color-scheme: dark;
    --bg: #0f172a;
    --panel: #182235;
    --panel2: #111827;
    --muted: #94a3b8;
    --line: #334155;
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
    color: #f8fafc;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
  }
  main { max-width: 1320px; margin: 0 auto; }
  h1 { margin: 0 0 8px; font-size: 27px; }
  .lead { color: var(--muted); margin: 0 0 18px; line-height: 1.55; }
  .summary {
    display: flex;
    flex-wrap: wrap;
    gap: 10px;
    margin-bottom: 22px;
  }
  .pill {
    padding: 7px 11px;
    border: 1px solid var(--line);
    border-radius: 999px;
    background: var(--panel2);
    color: #cbd5e1;
    font-size: 13px;
  }
  .legend {
    display: flex;
    flex-wrap: wrap;
    gap: 12px 18px;
    margin: 0 0 24px;
    color: #cbd5e1;
    font-size: 13px;
  }
  .legend span { display: inline-flex; align-items: center; gap: 6px; }
  .sw { width: 14px; height: 14px; border-radius: 3px; display: inline-block; }
  .matrix-head, .matrix-row {
    display: grid;
    grid-template-columns: 110px repeat(3, minmax(250px, 1fr));
    gap: 14px;
  }
  .matrix-head { margin-bottom: 10px; color: #cbd5e1; font-weight: 600; }
  .matrix-head > div { text-align: center; }
  .matrix-head > div:first-child { text-align: left; }
  .matrix-row { margin-bottom: 16px; align-items: stretch; }
  .row-label {
    display: flex;
    align-items: center;
    justify-content: center;
    text-align: center;
    border: 1px solid var(--line);
    background: var(--panel2);
    border-radius: 10px;
    font-weight: 700;
    line-height: 1.5;
  }
  .row-label small { display: block; color: var(--muted); font-weight: 400; }
  .card {
    background: var(--panel);
    border: 1px solid var(--line);
    border-radius: 12px;
    padding: 13px;
    min-width: 0;
  }
  .card canvas {
    display: block;
    width: min(100%, 250px);
    aspect-ratio: 1;
    margin: 0 auto 10px;
    border-radius: 6px;
    background: #0b1220;
  }
  .card h2 { font-size: 14px; margin: 0 0 6px; overflow-wrap: anywhere; }
  .meta { color: #cbd5e1; font-size: 12px; line-height: 1.55; }
  .links { display: flex; gap: 12px; margin-top: 9px; font-size: 13px; }
  a { color: #60a5fa; text-decoration: none; }
  a:hover { text-decoration: underline; }
  .note {
    margin-top: 22px;
    padding: 13px 15px;
    background: #2b2418;
    border-left: 3px solid #f59e0b;
    color: #fde68a;
    line-height: 1.55;
    font-size: 13px;
  }
  @media (max-width: 980px) {
    .matrix-head { display: none; }
    .matrix-row { grid-template-columns: 1fr; }
    .row-label { padding: 10px; }
  }
</style>
</head>
<body>
<main>
  <h1>20×20 Amazon 风格 Warehouse-block testcase suite</h1>
  <p class="lead">同一张 20×20 地图上比较 3 种 block size 与 3 档货架密度。每个 testcase 内所有 block 尺寸一致、每个 block 的货架数量一致，block 之间的通道宽度固定为 1。</p>
  <div class="summary">
    __RUN_LINK__
    <span class="pill">9 个实际 YAML testcase</span>
    <span class="pill">Block size：3×3 / 4×4 / 9×9</span>
    <span class="pill">目标密度：25% / 50% / 75%</span>
    <span class="pill">8 robots / 12 relocation targets</span>
    <span class="pill">seed = 0</span>
  </div>
  <div class="legend">
    <span><i class="sw" style="background:var(--aisle)"></i>通道</span>
    <span><i class="sw" style="background:var(--storage)"></i>空 storage slot</span>
    <span><i class="sw" style="background:var(--shelf)"></i>匿名货架</span>
    <span><i class="sw" style="background:var(--target)"></i>目标货架</span>
    <span><i class="sw" style="border:2px solid var(--goal)"></i>目标空位</span>
    <span><i class="sw" style="background:var(--robot);border-radius:50%"></i>机器人</span>
  </div>
  <div class="matrix-head">
    <div>Block size</div>
    <div>目标密度 25%</div>
    <div>目标密度 50%</div>
    <div>目标密度 75%</div>
  </div>
  __ROWS__
  <div class="note">通道禁放约束在这些 testcase 中定义为：初始货架、target start 和 target goal 均不得位于通道。现有 DD 动作模型仍允许求解过程中的临时 DROP；若要把它升级为运行时硬约束，需要同步加入跨通道连续搬运宏动作。</div>
</main>
<script>
const CASES = __DATA__;
const key = p => `${p[0]},${p[1]}`;

function drawCase(canvas, D) {
  const size = 250;
  const cell = size / D.width;
  const dpr = window.devicePixelRatio || 1;
  canvas.width = size * dpr;
  canvas.height = size * dpr;
  const ctx = canvas.getContext("2d");
  ctx.scale(dpr, dpr);
  const storage = new Set(D.storage.map(key));
  const target = new Set(D.targets.map(t => key(t.start)));

  for (let r = 0; r < D.height; r++) {
    for (let c = 0; c < D.width; c++) {
      ctx.fillStyle = storage.has(`${r},${c}`) ? "#d9c7a2" : "#26384a";
      ctx.fillRect(c * cell, r * cell, cell + .2, cell + .2);
    }
  }
  const shelfPad = Math.max(1, cell * .20);
  for (const p of D.shelves) {
    ctx.fillStyle = target.has(key(p)) ? "#f59e0b" : "#7c4a21";
    ctx.fillRect(
      p[1] * cell + shelfPad,
      p[0] * cell + shelfPad,
      cell - shelfPad * 2,
      cell - shelfPad * 2
    );
  }
  ctx.strokeStyle = "#22c55e";
  ctx.lineWidth = Math.max(1, cell * .13);
  for (const t of D.targets) {
    ctx.strokeRect(
      t.goal[1] * cell + 1,
      t.goal[0] * cell + 1,
      cell - 2,
      cell - 2
    );
  }
  ctx.fillStyle = "#3b82f6";
  for (const p of D.robots) {
    ctx.beginPath();
    ctx.arc(
      p[1] * cell + cell / 2,
      p[0] * cell + cell / 2,
      Math.max(1.7, cell * .30),
      0,
      Math.PI * 2
    );
    ctx.fill();
  }
}

document.querySelectorAll("canvas[data-case]").forEach(canvas => {
  drawCase(canvas, CASES[Number(canvas.dataset.case)]);
});
</script>
</body>
</html>
"""


def _dashboard_case(case: Dict) -> Dict:
    return {
        "name": case["name"],
        "height": case["height"],
        "width": case["width"],
        "storage": sorted(case["storage"]),
        "shelves": case["shelves"],
        "robots": case["robots"],
        "targets": case["targets"],
    }


def _manifest_row(case: Dict) -> Dict:
    period = case["block_size"] + case["aisle_width"]
    name = case["name"]
    return {
        "name": name,
        "height": case["height"],
        "width": case["width"],
        "block_size": case["block_size"],
        "aisle_width": case["aisle_width"],
        "block_rows": case["height"] // period,
        "block_cols": case["width"] // period,
        "block_count": len(case["blocks"]),
        "requested_density": f"{case['requested_density']:.4f}",
        "actual_density": f"{case['density']:.8f}",
        "shelves_per_block": case["shelves_per_block"],
        "total_shelves": len(case["shelves"]),
        "storage_cells": len(case["storage"]),
        "corridor_cells": len(case["corridor"]),
        "robots": len(case["robots"]),
        "targets": len(case["targets"]),
        "seed": SEED,
        "yaml": f"instances/{name}.yaml",
        "html": f"cases/{name}.html",
    }


def write_dashboard(
    cases: Sequence[Dict], path: Path, suite_dir_name: str
) -> None:
    rows: List[str] = []
    case_index = 0
    for block_size in BLOCK_SIZES:
        cards: List[str] = []
        block_cases = [
            case for case in cases if case["block_size"] == block_size
        ]
        for case in block_cases:
            actual = case["density"] * 100
            requested = case["requested_density"] * 100
            name = case["name"]
            cards.append(
                f"""<article class="card">
  <canvas data-case="{case_index}"></canvas>
  <h2>{name}</h2>
  <div class="meta">实际密度 {actual:.1f}%（目标 {requested:.0f}%） · 每块 {case['shelves_per_block']}/{block_size * block_size} · 总货架 {len(case['shelves'])} · {len(case['blocks'])} blocks</div>
  <div class="links">
    <a href="{suite_dir_name}/cases/{name}.html">打开详情</a>
    <a href="{suite_dir_name}/instances/{name}.yaml">下载 YAML</a>
  </div>
</article>"""
            )
            case_index += 1
        rows.append(
            f"""<section class="matrix-row">
  <div class="row-label"><div>{block_size}×{block_size}<small>{len(block_cases[0]['blocks'])} blocks</small></div></div>
  {''.join(cards)}
</section>"""
        )

    data = [_dashboard_case(case) for case in cases]
    run_report = (
        path.parent
        / suite_dir_name
        / "runs_lacam_10s_seed0"
        / "index.html"
    )
    run_link = (
        '<a class="pill" '
        'href="warehouse_block_suite/runs_lacam_10s_seed0/index.html">'
        "▶ 查看 LaCAM 9/9 实跑结果与动画</a>"
        if run_report.is_file()
        else ""
    )
    html = DASHBOARD_TEMPLATE.replace("__ROWS__", "\n".join(rows))
    html = html.replace("__RUN_LINK__", run_link)
    html = html.replace(
        "__DATA__", json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(html, encoding="utf-8")


def write_readme(rows: Sequence[Dict], path: Path) -> None:
    table = [
        "| block | target density | actual density | blocks | shelves/block | total shelves | YAML |",
        "|---:|---:|---:|---:|---:|---:|---|",
    ]
    for row in rows:
        table.append(
            f"| {row['block_size']}×{row['block_size']} "
            f"| {float(row['requested_density']):.0%} "
            f"| {float(row['actual_density']):.1%} "
            f"| {row['block_count']} "
            f"| {row['shelves_per_block']} "
            f"| {row['total_shelves']} "
            f"| `{row['yaml']}` |"
        )
    table_text = "\n".join(table)
    text = f"""# 20×20 warehouse-block testcase suite

- Map size: {HEIGHT}×{WIDTH}
- Aisle width: {AISLE_WIDTH}
- Block sizes: {", ".join(str(v) for v in BLOCK_SIZES)}
- Requested density levels: {", ".join(f"{v:.0%}" for v in DENSITY_LEVELS)}
- Robots / relocation targets: {ROBOTS} / {TARGETS}
- Seed: {SEED}

Every testcase has one block size and exactly the same shelf count in every
block. Initial shelves, target starts, and target goals are storage cells;
robots start in aisle cells. The current DD action schema has no runtime
drop-zone mask, so corridor exclusion is a generated-layout contract.

{table_text}
"""
    path.write_text(text, encoding="utf-8")


def generate_suite(output_dir: Path, dashboard_path: Path) -> List[Dict]:
    instances_dir = output_dir / "instances"
    cases_dir = output_dir / "cases"
    instances_dir.mkdir(parents=True, exist_ok=True)
    cases_dir.mkdir(parents=True, exist_ok=True)

    cases: List[Dict] = []
    rows: List[Dict] = []
    for block_size in BLOCK_SIZES:
        for density in DENSITY_LEVELS:
            case = build_case(
                height=HEIGHT,
                width=WIDTH,
                block_size=block_size,
                aisle_width=AISLE_WIDTH,
                density=density,
                n_robots=ROBOTS,
                n_targets=TARGETS,
                seed=SEED,
            )
            yaml_path = instances_dir / f"{case['name']}.yaml"
            html_path = cases_dir / f"{case['name']}.html"
            write_yaml(case, yaml_path)
            write_html(case, html_path)

            loaded = load_instance(yaml_path)
            errors = loaded.validate_static()
            if errors:
                raise ValueError(f"{case['name']} failed DD validation: {errors}")
            if not set(loaded.shelves) <= case["storage"]:
                raise ValueError(f"{case['name']} placed a shelf in an aisle")
            if not all(
                tuple(target.start) in case["storage"]
                and tuple(target.goal) in case["storage"]
                for target in loaded.targets
            ):
                raise ValueError(
                    f"{case['name']} placed a target endpoint in an aisle"
                )

            cases.append(case)
            rows.append(_manifest_row(case))

    manifest_csv = output_dir / "manifest.csv"
    with manifest_csv.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(rows[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)
    (output_dir / "manifest.json").write_text(
        json.dumps(rows, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    write_readme(rows, output_dir / "README.md")
    write_dashboard(cases, dashboard_path, output_dir.name)
    return rows


def parse_args() -> argparse.Namespace:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=here / "viz_web" / "warehouse_block_suite",
    )
    parser.add_argument(
        "--dashboard",
        type=Path,
        default=here / "viz_web" / "warehouse_block_suite.html",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rows = generate_suite(args.output_dir, args.dashboard)
    print(f"dashboard={args.dashboard}")
    print(f"suite={args.output_dir}")
    print(
        f"cases={len(rows)}, block_sizes={list(BLOCK_SIZES)}, "
        f"density_levels={[int(v * 100) for v in DENSITY_LEVELS]}, "
        f"aisle_width={AISLE_WIDTH}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
