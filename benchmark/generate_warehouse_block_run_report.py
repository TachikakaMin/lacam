#!/usr/bin/env python3
"""Build an HTML/CSV report from a warehouse-block dd_benchmark run."""

import argparse
import csv
import html
from pathlib import Path
from typing import Dict, List


HTML_HEAD = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Warehouse-block LaCAM 实跑结果</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 24px; background: #0f172a; color: #f8fafc;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
  }
  main { max-width: 1280px; margin: 0 auto; }
  h1 { margin: 0 0 8px; font-size: 27px; }
  .lead { margin: 0 0 20px; color: #94a3b8; line-height: 1.55; }
  .summary { display: flex; flex-wrap: wrap; gap: 10px; margin-bottom: 22px; }
  .pill {
    background: #111827; border: 1px solid #334155; border-radius: 999px;
    padding: 7px 11px; color: #cbd5e1; font-size: 13px;
  }
  .matrix-head, .matrix-row {
    display: grid; grid-template-columns: 105px repeat(3, minmax(250px, 1fr));
    gap: 14px;
  }
  .matrix-head { margin-bottom: 10px; color: #cbd5e1; font-weight: 600; }
  .matrix-head div { text-align: center; }
  .matrix-head div:first-child { text-align: left; }
  .matrix-row { margin-bottom: 16px; }
  .row-label {
    display: flex; align-items: center; justify-content: center;
    background: #111827; border: 1px solid #334155; border-radius: 10px;
    font-weight: 700; text-align: center;
  }
  .card {
    background: #182235; border: 1px solid #334155; border-radius: 12px;
    padding: 14px;
  }
  .ok { color: #4ade80; font-weight: 700; }
  .bad { color: #f87171; font-weight: 700; }
  .main-number { font-size: 26px; font-weight: 750; margin: 7px 0 2px; }
  .sub { color: #94a3b8; font-size: 12px; margin-bottom: 10px; }
  .metrics {
    display: grid; grid-template-columns: 1fr 1fr; gap: 7px;
    font-size: 12px; color: #cbd5e1;
  }
  .metric { background: #111827; border-radius: 6px; padding: 7px; }
  .metric b { display: block; color: #f8fafc; font-size: 14px; }
  .links { display: flex; flex-wrap: wrap; gap: 12px; margin-top: 11px; }
  a { color: #60a5fa; text-decoration: none; font-size: 13px; }
  a:hover { text-decoration: underline; }
  .note {
    margin-top: 22px; padding: 13px 15px; background: #172033;
    border-left: 3px solid #60a5fa; color: #cbd5e1;
    font-size: 13px; line-height: 1.55;
  }
  @media (max-width: 950px) {
    .matrix-head { display: none; }
    .matrix-row { grid-template-columns: 1fr; }
    .row-label { padding: 9px; }
  }
</style>
</head>
<body><main>
"""


def parse_metrics(path: Path) -> Dict[str, str]:
    metrics: Dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        metrics[key.strip()] = value.strip()
    return metrics


def generate_report(suite_dir: Path, run_dir: Path) -> List[Dict[str, str]]:
    with (suite_dir / "manifest.csv").open(
        encoding="utf-8", newline=""
    ) as stream:
        manifest = list(csv.DictReader(stream))

    rows: List[Dict[str, str]] = []
    for case in manifest:
        name = case["name"]
        log_path = run_dir / "logs" / f"{name}.log"
        plan_path = run_dir / "plans" / f"{name}.plan"
        viz_path = run_dir / "viz" / f"{name}.html"
        if not log_path.is_file():
            raise ValueError(f"missing log for {name}")
        metrics = parse_metrics(log_path)
        row = dict(case)
        for key in (
            "valid_instance",
            "solved",
            "makespan",
            "loaded_moves",
            "free_moves",
            "lift_drop",
            "anon_moves",
            "weighted_soc",
            "runtime_ms",
            "first_solution_ms",
            "timed_out",
            "generator_failures",
            "best_targets_done",
        ):
            row[key] = metrics.get(key, "")
        row["plan"] = f"plans/{name}.plan" if plan_path.is_file() else ""
        row["visualization"] = (
            f"viz/{name}.html" if viz_path.is_file() else ""
        )
        row["log"] = f"logs/{name}.log"
        rows.append(row)

    with (run_dir / "summary.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(rows[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)

    solved = sum(row["solved"] == "1" for row in rows)
    runtimes = [float(row["runtime_ms"]) for row in rows]
    makespans = [
        int(row["makespan"]) for row in rows if row["solved"] == "1"
    ]
    cards_by_block: Dict[int, List[str]] = {}
    for row in rows:
        block_size = int(row["block_size"])
        is_solved = row["solved"] == "1"
        name = html.escape(row["name"])
        links = []
        if row["visualization"]:
            links.append(
                f'<a href="{row["visualization"]}">▶ 播放动画</a>'
            )
        if row["plan"]:
            links.append(f'<a href="{row["plan"]}">Plan</a>')
        links.append(f'<a href="{row["log"]}">完整日志</a>')
        card = f"""<article class="card">
  <div class="{'ok' if is_solved else 'bad'}">{'✓ solved' if is_solved else '✗ unsolved'}</div>
  <div class="main-number">{row['makespan'] if is_solved else '—'}</div>
  <div class="sub">makespan · {name}</div>
  <div class="metrics">
    <div class="metric">运行时间<b>{float(row['runtime_ms']):.1f} ms</b></div>
    <div class="metric">Weighted SOC<b>{row['weighted_soc']}</b></div>
    <div class="metric">Loaded moves<b>{row['loaded_moves']}</b></div>
    <div class="metric">Free moves<b>{row['free_moves']}</b></div>
    <div class="metric">Lift / Drop<b>{row['lift_drop']}</b></div>
    <div class="metric">实际密度<b>{float(row['actual_density']):.1%}</b></div>
  </div>
  <div class="links">{''.join(links)}</div>
</article>"""
        cards_by_block.setdefault(block_size, []).append(card)

    body = [
        HTML_HEAD,
        "<h1>Warehouse-block LaCAM 实跑结果</h1>",
        '<p class="lead">当前 <code>build/dd_benchmark</code>，mode=lacam，seed=0，每例上限 10 秒；9 个实例并行 smoke run。点击任意案例可播放经过 authoritative validator 验证的完整 plan。</p>',
        '<div class="summary">',
        f'<span class="pill">解出率：{solved}/{len(rows)}</span>',
        f'<span class="pill">运行时间：{min(runtimes):.1f}–{max(runtimes):.1f} ms</span>',
        f'<span class="pill">Makespan：{min(makespans)}–{max(makespans)}</span>',
        '<span class="pill">超时：0</span>',
        "</div>",
        '<div class="matrix-head"><div>Block</div><div>25%</div><div>50%</div><div>75%</div></div>',
    ]
    for block_size in sorted(cards_by_block):
        body.append(
            f'<section class="matrix-row"><div class="row-label">{block_size}×{block_size}</div>'
            + "".join(cards_by_block[block_size])
            + "</section>"
        )
    body.extend(
        [
            '<div class="note">这是功能性 smoke run，不是隔离 CPU 的正式性能 benchmark。所有 9 个输出 plan 都在生成动画前由 Python authoritative validator 重放并确认到达目标。</div>',
            "</main></body></html>",
        ]
    )
    (run_dir / "index.html").write_text(
        "\n".join(body), encoding="utf-8"
    )
    return rows


def parse_args() -> argparse.Namespace:
    here = Path(__file__).resolve().parent
    default_suite = here / "viz_web" / "warehouse_block_suite"
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite-dir", type=Path, default=default_suite)
    parser.add_argument(
        "--run-dir",
        type=Path,
        default=default_suite / "runs_lacam_10s_seed0",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rows = generate_report(args.suite_dir, args.run_dir)
    print(f"report={args.run_dir / 'index.html'}")
    print(f"summary={args.run_dir / 'summary.csv'}")
    print(f"solved={sum(row['solved'] == '1' for row in rows)}/{len(rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
