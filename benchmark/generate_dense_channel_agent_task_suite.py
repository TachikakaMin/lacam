#!/usr/bin/env python3
"""Generate the frozen 40x40 b9 dense-channel agent-by-task suite."""

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
BLOCK_SIZE = 9
SHELVES_PER_BLOCK = 76
INSTANCE_SEED = 1
AGENT_COUNTS = (4, 8, 12, 16, 24, 32, 48, 64)
TARGET_COUNTS = (24, 48, 96)


def build_case_specs() -> List[Dict]:
    return [
        {
            "agent_level": "r{}".format(robots),
            "task_level": "t{}".format(targets),
            "robots": robots,
            "targets": targets,
            "block_size": BLOCK_SIZE,
            "shelves_per_block": SHELVES_PER_BLOCK,
            "seed": INSTANCE_SEED,
        }
        for robots in AGENT_COUNTS
        for targets in TARGET_COUNTS
    ]


def _write_index(rows: List[Dict], output_dir: Path) -> None:
    cards = []
    for row in rows:
        cards.append(
            """<article><h2>{name}</h2>
<p><b>{robots}</b> robots · <b>{targets}</b> targets · 76/81 shelves ·
每个 target 有 32 个起始 block 边缘目标 · witness {steps} 拍</p>
<p><a href="{preview}">初始布局</a> · <a href="{yaml}">YAML</a> ·
<a href="{certificate}">可解性 witness</a></p></article>""".format(
                name=html.escape(row["name"]),
                robots=row["robots"],
                targets=row["targets"],
                steps=row["certificate_steps"],
                preview=html.escape(row["preview"]),
                yaml=html.escape(row["yaml"]),
                certificate=html.escape(row["certificate"]),
            )
        )
    page = """<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>40×40 b9 Dense-channel Agent×Task V1</title><style>
body{{margin:auto;max-width:1280px;padding:30px;background:#111315;color:#f1f3f4;
font:15px/1.65 system-ui}}a{{color:#79b7e8}}.lead{{color:#aab0b5}}
.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(350px,1fr));
gap:14px}}article{{border-top:1px solid #393d41;padding:14px 0}}
h2{{font-size:14px;overflow-wrap:anywhere}}b{{color:#8fd3ff}}
</style></head><body><h1>40×40 b9 Dense-channel：Agent×Task 扩展</h1>
<p class="lead">固定 9×9 storage block、76/81 货架和单格通道；机器人数量
4、8、12、16、24、32、48、64，目标任务数 24、48、96，共 24 个配置。
每个目标从 block 内层出发，最终只能选择其起始 block 的 32 个边缘 storage。
逆向 witness 仅证明实例可解，solver 不读取。</p><div class="grid">{cards}</div>
</body></html>""".format(cards="\n".join(cards))
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
            seed=spec["seed"],
            target_profile=INTERIOR_TO_BLOCK_EDGE_SET,
            height=HEIGHT,
            width=WIDTH,
            n_robots=spec["robots"],
            n_targets=spec["targets"],
        )
        generated["agent_level"] = spec["agent_level"]
        ins = generated["instance"]
        save_instance(ins, instances_dir / "{}.yaml".format(ins.name))
        (certificates_dir / "{}.plan".format(ins.name)).write_text(
            _plan_text(generated["witness"]), encoding="utf-8"
        )
        write_html(generated["case"], cases_dir / "{}.html".format(ins.name))
        row = _manifest_row(generated)
        row["agent_level"] = spec["agent_level"]
        row["task_level"] = spec["task_level"]
        rows.append(row)

    if len(rows) != len(AGENT_COUNTS) * len(TARGET_COUNTS):
        raise RuntimeError("agent-by-task suite cardinality changed")
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
            / "dense_channel_b9_agent_task_suite_v1_20260914"
        ),
    )
    args = parser.parse_args()
    rows = generate_suite(args.output_dir)
    print(
        "generated {} b9 agent-by-task dense-channel cases under {}".format(
            len(rows), args.output_dir
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
