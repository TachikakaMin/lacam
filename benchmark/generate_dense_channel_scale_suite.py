#!/usr/bin/env python3
"""Generate the 40x40 dense-channel block-edge goal-set scaling suite."""

import argparse
import html
import json
from pathlib import Path
from typing import Dict, List

from ddbench.instance import save_instance
from generate_dense_channel_suite import (
    INTERIOR_TO_BLOCK_EDGE_SET,
    _manifest_row,
    _plan_text,
    build_dense_case,
)
from generate_warehouse_block_sample import write_html


HEIGHT = 40
WIDTH = 40
LOAD_LEVELS = (
    ("small", 8, 12),
    ("medium", 16, 24),
    ("large", 32, 48),
)
HIGH_DENSITY_COUNTS = {
    3: 8,
    4: 15,
    9: 76,
}


def build_case_specs() -> List[Dict]:
    return [
        {
            "load_level": load_level,
            "robots": robots,
            "targets": targets,
            "block_size": block_size,
            "shelves_per_block": HIGH_DENSITY_COUNTS[block_size],
        }
        for block_size in sorted(HIGH_DENSITY_COUNTS)
        for load_level, robots, targets in LOAD_LEVELS
    ]


def _write_index(rows: List[Dict], output_dir: Path) -> None:
    cards = []
    for row in rows:
        cards.append(
            """<article><h2>{name}</h2>
<p>{block}×{block} block · 密度 <b>{density:.1%}</b> ·
{robots} robots / {targets} targets · 每个 target 有
{goals} 个最终候选（均为起始 block edge）· witness {steps} 拍</p>
<p><a href="{preview}">查看初始布局</a> ·
<a href="{yaml}">YAML</a> ·
<a href="{certificate}">可解性 witness</a></p></article>""".format(
                name=html.escape(row["name"]),
                block=row["block_size"],
                density=row["actual_density"],
                robots=row["robots"],
                targets=row["targets"],
                goals=row["goals_per_target"],
                steps=row["certificate_steps"],
                preview=html.escape(row["preview"]),
                yaml=html.escape(row["yaml"]),
                certificate=html.escape(row["certificate"]),
            )
        )
    page = """<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>40×40 Dense-channel block-edge scaling V1</title><style>
body{{margin:auto;max-width:1180px;padding:30px;background:#111315;color:#f1f3f4;
font:15px/1.65 system-ui}}a{{color:#79b7e8}}.lead{{color:#aab0b5}}
.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(330px,1fr));
gap:14px}}article{{border-top:1px solid #393d41;padding:14px 0}}
h2{{font-size:15px;overflow-wrap:anywhere}}b{{color:#f1f3f4}}
</style></head><body><h1>40×40 Dense-channel：终点为起始 block 任一边缘</h1>
<p class="lead">三档负载分别为 8/12、16/24、32/48（机器人/目标）。
每个目标货箱从 block 内部出发，最终 goal 可选择其起始 block 的任意
edge storage。这只约束终态；运输途中可经过其他 block，也可在其他合法
storage 临时落箱。生成 witness 只用于证明实例可解，solver 不读取。</p>
<div class="grid">{cards}</div></body></html>""".format(
        cards="\n".join(cards)
    )
    (output_dir / "index.html").write_text(page, encoding="utf-8")


def generate_suite(output_dir: Path) -> List[Dict]:
    output_dir = Path(output_dir)
    instances_dir = output_dir / "instances"
    certificates_dir = output_dir / "certificates"
    cases_dir = output_dir / "cases"
    for directory in (instances_dir, certificates_dir, cases_dir):
        directory.mkdir(parents=True, exist_ok=True)

    rows = []
    for spec in build_case_specs():
        generated = build_dense_case(
            block_size=spec["block_size"],
            shelves_per_block=spec["shelves_per_block"],
            seed=0,
            target_profile=INTERIOR_TO_BLOCK_EDGE_SET,
            height=HEIGHT,
            width=WIDTH,
            n_robots=spec["robots"],
            n_targets=spec["targets"],
        )
        generated["agent_level"] = spec["load_level"]
        ins = generated["instance"]
        save_instance(
            ins, instances_dir / "{}.yaml".format(ins.name)
        )
        certificate_path = (
            certificates_dir / "{}.plan".format(ins.name)
        )
        certificate_path.write_text(
            _plan_text(generated["witness"]), encoding="utf-8"
        )
        write_html(
            generated["case"],
            cases_dir / "{}.html".format(ins.name),
        )
        row = _manifest_row(generated)
        row["load_level"] = spec["load_level"]
        rows.append(row)

    (output_dir / "manifest.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    _write_index(rows, output_dir)
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            Path(__file__).resolve().parent
            / "viz_web"
            / "dense_channel_block_edge_40x40_suite_v1_20260907"
        ),
    )
    args = parser.parse_args()
    rows = generate_suite(args.output_dir)
    print(
        "generated {} 40x40 block-edge scale cases under {}".format(
            len(rows), args.output_dir
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
