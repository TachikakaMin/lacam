#!/usr/bin/env python3
"""Generate the Dense-channel V2 benchmark dashboard and animations."""

import argparse
import csv
import hashlib
import html
import json
import os
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path


BENCH = Path(__file__).resolve().parent
DEFAULT_SUITE = (
    BENCH / "viz_web" / "dense_channel_suite_v2_20260906"
)
DEFAULT_RESULTS = BENCH / "results_dense_channel_v2_20260906"
DEFAULT_OUTPUT = (
    BENCH / "viz_web" / "dense_channel_benchmark_v2_20260906"
)
VIZ_GENERATOR = BENCH / "generate_web_viz.py"
SUITE_URL = "../dense_channel_suite_v2_20260906/"


def _success(row):
    return str(row.get("success", "")).lower() in {
        "1", "true", "yes"
    }


def _number(row, key, cast=float):
    value = row.get(key, "")
    if value in ("", None):
        return None
    return cast(float(value)) if cast is int else cast(value)


def _fmt_seconds(milliseconds):
    if milliseconds is None:
        return "—"
    return "{:.3f} s".format(milliseconds / 1000.0)


def build_report_data(
    rows_path, timing_path, manifest_path, suite_url=SUITE_URL
):
    rows_path = Path(rows_path)
    timing_path = Path(timing_path)
    manifest_path = Path(manifest_path)
    with rows_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    timing = json.loads(timing_path.read_text(encoding="utf-8"))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    suite_url = suite_url.rstrip("/") + "/"
    metadata = {row["id"]: row for row in manifest}
    row_ids = {row["instance"] for row in rows}
    if row_ids != set(metadata):
        raise ValueError(
            "benchmark rows and dense-channel manifest differ: "
            "rows_only={} manifest_only={}".format(
                sorted(row_ids - set(metadata)),
                sorted(set(metadata) - row_ids),
            )
        )

    records = []
    for row in rows:
        meta = metadata[row["instance"]]
        solved = _success(row)
        record = {
            "instance": row["instance"],
            "family": row.get("family", "dense_channel"),
            "success": solved,
            "status": row.get("status", ""),
            "block_size": int(meta["block_size"]),
            "density_level": meta["density_level"],
            "actual_density": float(meta["actual_density"]),
            "shelves_per_block": int(
                meta.get(
                    "shelves_per_block",
                    str(meta["density_level"]).split("/", 1)[0],
                )
            ),
            "start_edge": int(meta["start_edge"]),
            "start_inner": int(meta["start_inner"]),
            "start_deep": int(meta["start_deep"]),
            "goal_edge": int(meta["goal_edge"]),
            "goal_inner": int(meta["goal_inner"]),
            "target_profile": meta.get(
                "task_profile", "depth_stratified"
            ),
            "robots": int(meta.get("robots", 0)),
            "targets": int(
                meta.get(
                    "targets",
                    int(meta["start_edge"]) + int(meta["start_inner"]),
                )
            ),
            "goals_per_target": int(meta.get("goals_per_target", 1)),
            "first_ms": _number(row, "first_solution_ms"),
            "return_ms": _number(row, "deliverable_ms"),
            "runtime_sec": _number(row, "runtime_sec"),
            "first_makespan": _number(
                row, "first_solution_makespan", int
            ),
            "final_makespan": _number(
                row, "executed_makespan", int
            ),
            "weighted_soc": _number(row, "weighted_soc"),
            "improvements": (
                _number(row, "improvement_improvements", int) or 0
            ),
            "exit_reason": row.get("improvement_exit_reason", ""),
            "plan_sha256": row.get("plan_sha256", ""),
            "animation": (
                "cases/{}.html".format(row["instance"])
                if solved else None
            ),
            "preview": suite_url + meta["preview"],
            "yaml": suite_url + meta["yaml"],
            "certificate": suite_url + meta["certificate"],
            "manifest": meta,
        }
        records.append(record)

    records.sort(
        key=lambda row: (
            row["block_size"],
            row["shelves_per_block"],
            row["robots"],
            row["targets"],
        )
    )
    solved = [row for row in records if row["success"]]
    summary = {
        "total": len(records),
        "solved": len(solved),
        "inner_starts": sum(row["start_inner"] for row in records),
        "deep_starts": sum(row["start_deep"] for row in records),
        "edge_goals": sum(row["goal_edge"] for row in records),
        "inner_goals": sum(row["goal_inner"] for row in records),
        "min_goals_per_target": min(
            (row["goals_per_target"] for row in records), default=0
        ),
        "max_goals_per_target": max(
            (row["goals_per_target"] for row in records), default=0
        ),
        "target_slots": sum(
            row["start_edge"] + row["start_inner"]
            for row in records
        ),
        "min_density": min(
            (row["actual_density"] for row in records), default=0
        ),
        "max_density": max(
            (row["actual_density"] for row in records), default=0
        ),
        "wall_time_sec": float(timing["wall_time_sec"]),
        "timeout_sec": float(timing["timeout_per_run_sec"]),
        "jobs": int(timing["jobs"]),
    }
    return {
        "records": records,
        "summary": summary,
        "timing": timing,
        "rows_path": str(rows_path),
        "timing_path": str(timing_path),
        "manifest_path": str(manifest_path),
    }


def _case_card(record, timeout_ms):
    state_class = "ok" if record["success"] else "bad"
    state_text = "✓ solved" if record["success"] else "✗ unsolved"
    first_width = min(
        100.0, 100.0 * (record["first_ms"] or 0) / timeout_ms
    )
    return_width = min(
        100.0, 100.0 * (record["return_ms"] or 0) / timeout_ms
    )
    if record["success"]:
        animation = (
            '<a class="primary" href="{}">▶ 播放 solver 动画</a>'.format(
                html.escape(record["animation"])
            )
        )
    else:
        animation = '<span class="disabled">无合法动画</span>'
    makespan = "首解 makespan {} → 最终 {}".format(
        record["first_makespan"], record["final_makespan"]
    )
    scale_tags = ""
    if record.get("robots") and record.get("targets"):
        scale_tags += (
            "\n    <span>{} robots</span>"
            "\n    <span>{} targets</span>"
        ).format(record["robots"], record["targets"])
    if record.get("goals_per_target", 1) > 1:
        scale_tags += "\n    <span>{} edge goals/target</span>".format(
            record["goals_per_target"]
        )
    return """<article class="case-card">
  <div class="case-top">
    <span class="state {state_class}">{state_text}</span>
    <span class="density">{density:.1%}</span>
  </div>
  <h3>{instance}</h3>
  <div class="tags">
    <span>{block}×{block} block</span>
    <span>{density_level} shelves</span>
    {scale_tags}
    <span>start: edge {start_edge} · inner {start_inner} · deep {start_deep}</span>
    <span>goal: edge {goal_edge} · inner {goal_inner}</span>
  </div>
  <div class="timing">
    <div class="timing-row">
      <div>首解 {first_time}</div>
      <div class="track"><span class="first" style="width:{first_width:.2f}%"></span></div>
    </div>
    <div class="timing-row">
      <div>最终返回 {return_time}</div>
      <div class="track"><span class="final" style="width:{return_width:.2f}%"></span></div>
    </div>
  </div>
  <div class="cost">{makespan}<br>Weighted SOC {soc:.0f} ·
    phase 2: {exit_reason}</div>
  <div class="links">
    {animation}
    <a href="{preview}">初始布局</a>
    <a href="{yaml}">YAML</a>
    <a href="{certificate}">生成见证</a>
  </div>
</article>""".format(
        state_class=state_class,
        state_text=state_text,
        density=record["actual_density"],
        instance=html.escape(record["instance"]),
        block=record["block_size"],
        density_level=html.escape(record["density_level"]),
        scale_tags=scale_tags,
        start_edge=record["start_edge"],
        start_inner=record["start_inner"],
        start_deep=record["start_deep"],
        goal_edge=record["goal_edge"],
        goal_inner=record["goal_inner"],
        first_time=_fmt_seconds(record["first_ms"]),
        return_time=_fmt_seconds(record["return_ms"]),
        first_width=first_width,
        return_width=return_width,
        makespan=html.escape(makespan),
        soc=record["weighted_soc"] or 0,
        exit_reason=html.escape(record["exit_reason"] or "—"),
        animation=animation,
        preview=html.escape(record["preview"]),
        yaml=html.escape(record["yaml"]),
        certificate=html.escape(record["certificate"]),
    )


def render_dashboard(data):
    summary = data["summary"]
    timing = data["timing"]
    profiles = {
        record.get("target_profile")
        for record in data["records"]
        if record.get("target_profile")
    }
    block_edge_set = profiles == {"interior_to_block_edge_set"}
    interior_to_edge = profiles == {"interior_to_edge"}
    if block_edge_set:
        page_title = (
            "40×40 Dense-channel benchmark · 起始 block 边缘终点"
        )
        heading = "40×40 Dense-channel：负载扩展"
        lead = (
            "目标货箱从 storage block 内部出发，最终 goal 可选择其起始 "
            "block 的任意边缘 storage 位置。这个集合只约束终态；运输"
            "途中可经过其他 block，也可在其他合法 storage 临时落箱。"
            "机器人/目标按 8/12、16/24、32/48 三档递增。"
        )
        goal_label = "每目标 edge 候选"
        goal_value = "{}–{}".format(
            summary["min_goals_per_target"],
            summary["max_goals_per_target"],
        )
    elif interior_to_edge:
        page_title = "Dense-channel benchmark · 内部货箱到边缘目标"
        heading = "Dense-channel：内部货箱到边缘目标"
        lead = (
            "这套高密度通道压力图要求所有目标货箱从 storage block "
            "内部出发，并运到紧邻通道的 block 边缘。生成见证只证明实例"
            "可解，Carrier-LaCAM benchmark 不会读取见证。"
        )
        goal_label = "边缘 goals"
        goal_value = "{} / {}".format(
            summary["edge_goals"], summary["target_slots"]
        )
    else:
        page_title = "Dense-channel benchmark V2 · 高密度与内层目标"
        heading = "Dense-channel benchmark V2"
        lead = (
            "这是独立于已发布 509 例的新压力集。旧通道实例的目标起点 "
            "93.5% 位于 block 边缘；本套件提高实际货架密度，并明确按 "
            "edge / inner / deep 分层选择目标。生成见证只证明实例可解，"
            "Carrier-LaCAM benchmark 不会读取见证。"
        )
        goal_label = "内层 goals"
        goal_value = "{} / {}".format(
            summary["inner_goals"], summary["target_slots"]
        )
    timeout_ms = max(1.0, summary["timeout_sec"] * 1000.0)
    groups = {}
    for record in data["records"]:
        groups.setdefault(record["block_size"], []).append(record)
    sections = []
    for block_size in sorted(groups):
        sections.append(
            """<section class="panel">
  <div class="section-title"><h2>{block}×{block} storage blocks</h2>
  <span>{count} 个密度档</span></div>
  <div class="case-grid">{cards}</div>
</section>""".format(
                block=block_size,
                count=len(groups[block_size]),
                cards="\n".join(
                    _case_card(record, timeout_ms)
                    for record in groups[block_size]
                ),
            )
        )

    provenance = timing.get("provenance", {})
    suite = timing.get("suite", {})
    template = """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{page_title}</title>
<style>
:root {{
  color-scheme:dark; --bg:#07111e; --panel:#101d30; --panel2:#0a1626;
  --line:#29405c; --text:#eef6ff; --muted:#94a8bf; --blue:#4b9dff;
  --green:#47d7a6; --amber:#f5bc58; --red:#ff7585;
}}
* {{ box-sizing:border-box; }}
body {{
  margin:0; color:var(--text); font:14px/1.55 system-ui,sans-serif;
  background:
    radial-gradient(circle at 8% 0,#173968 0,transparent 30rem),
    radial-gradient(circle at 92% 4%,#342446 0,transparent 29rem),var(--bg);
}}
main {{ max-width:1460px; margin:auto; padding:28px; }}
h1 {{ margin:0; font-size:30px; }}
h2 {{ margin:0; font-size:20px; }}
h3 {{
  margin:9px 0 11px; font:600 13px/1.45 ui-monospace,monospace;
  overflow-wrap:anywhere;
}}
a {{ color:#75b6ff; text-decoration:none; }}
a:hover {{ text-decoration:underline; }}
.lead {{ max-width:1050px; color:var(--muted); font-size:15px; }}
.cards {{ display:grid; grid-template-columns:repeat(6,1fr); gap:11px; }}
.summary,.panel,.case-card {{
  border:1px solid var(--line); border-radius:14px;
  background:linear-gradient(145deg,#122139f5,#0b1729f5);
}}
.summary {{ padding:15px; }}
.summary .label {{ color:var(--muted); font-size:11px; letter-spacing:.07em; }}
.summary .value {{ margin-top:7px; font-size:25px; font-weight:780; }}
.panel {{ margin-top:16px; padding:18px; }}
.audit {{ display:grid; grid-template-columns:1.35fr .65fr; gap:16px; }}
.audit p {{ color:var(--muted); margin:5px 0; }}
.section-title {{
  display:flex; align-items:center; justify-content:space-between; margin-bottom:13px;
}}
.section-title span {{ color:var(--muted); }}
.case-grid {{
  display:grid; grid-template-columns:repeat(auto-fit,minmax(350px,1fr)); gap:12px;
}}
.case-card {{ padding:15px; background:var(--panel2); }}
.case-top {{ display:flex; justify-content:space-between; align-items:center; }}
.state {{ font-weight:750; font-size:12px; }}
.state.ok {{ color:var(--green); }} .state.bad {{ color:var(--red); }}
.density {{ color:#09101b; background:var(--amber); border-radius:999px;
  padding:3px 8px; font-weight:800; }}
.tags {{ display:flex; flex-wrap:wrap; gap:6px; }}
.tags span {{ color:#bfd0e3; background:#14243a; border:1px solid #263d59;
  border-radius:999px; padding:3px 7px; font-size:11px; }}
.timing {{ margin:14px 0 10px; }}
.timing-row {{
  display:grid; grid-template-columns:130px 1fr; gap:9px; align-items:center;
  color:#c8d6e7; font-size:12px; margin:7px 0;
}}
.track {{ height:10px; border-radius:999px; overflow:hidden;
  background:#18243a; border:1px solid #2b405c; }}
.track span {{ display:block; height:100%; min-width:2px; border-radius:999px; }}
.track .first {{ background:var(--green); }}
.track .final {{ background:linear-gradient(90deg,#337ee7,#9a63ed); }}
.cost {{ color:var(--muted); background:#0d1929; border-radius:9px;
  padding:9px; font-size:12px; }}
.links {{ display:flex; flex-wrap:wrap; gap:12px; margin-top:11px; }}
.primary {{ color:white; background:#1d65ba; padding:5px 9px; border-radius:7px; }}
.disabled {{ color:#708198; }}
code {{ color:#d7e7fb; overflow-wrap:anywhere; }}
.foot {{ color:var(--muted); font-size:12px; }}
@media(max-width:1050px) {{
  .cards {{ grid-template-columns:repeat(3,1fr); }} .audit {{ grid-template-columns:1fr; }}
}}
@media(max-width:650px) {{
  main {{ padding:16px; }} .cards {{ grid-template-columns:repeat(2,1fr); }}
  .case-grid {{ grid-template-columns:1fr; }}
}}
</style>
</head>
<body><main>
<h1>{heading}</h1>
<p class="lead">{lead}</p>

<section class="cards">
  <article class="summary"><div class="label">解出率</div>
    <div class="value">{solved} / {total}</div></article>
  <article class="summary"><div class="label">密度范围</div>
    <div class="value">{min_density:.1%}–{max_density:.1%}</div></article>
  <article class="summary"><div class="label">内层起点</div>
    <div class="value">{inner_starts} / {target_slots}</div></article>
  <article class="summary"><div class="label">深层起点</div>
    <div class="value">{deep_starts} / {target_slots}</div></article>
  <article class="summary"><div class="label">{goal_label}</div>
    <div class="value">{goal_value}</div></article>
  <article class="summary"><div class="label">并行 wall time</div>
    <div class="value">{wall:.1f} s</div></article>
</section>

<section class="panel audit">
  <div>
    <h2>怎样读时间条</h2>
    <p>绿色是 kernel 原始首解时刻：搜索第一次到达 goal，但尚未经过 normalize、
    repair 和最终验证；紫色是第二遍严格改进搜索、验证和清理完成后的最终返回
    时刻。两条都以本例 10 秒预算为横轴，它们不是机器人执行计划的 makespan。</p>
  </div>
  <div>
    <p>Suite：<code>{suite_name}</code></p>
    <p>Commit：<code>{commit}</code></p>
    <p>Binary SHA：<code>{binary_sha}</code></p>
    <p><a href="rows.csv">rows.csv</a> ·
       <a href="timing.json">timing.json</a> ·
       <a href="summary.json">summary.json</a></p>
  </div>
</section>

{sections}

<p class="foot">协议：carrier · seed 0 · unit weights · following allowed ·
单例上限 {timeout:.0f} 秒 · {jobs} 个并行 worker。动画只为 rows.csv 中
success=1 且 plan SHA 匹配的计划生成，生成工具还会调用 authoritative
validator 完整重放。</p>
</main></body></html>""".format(
        page_title=html.escape(page_title),
        heading=html.escape(heading),
        lead=html.escape(lead),
        solved=summary["solved"],
        total=summary["total"],
        min_density=summary["min_density"],
        max_density=summary["max_density"],
        inner_starts=summary["inner_starts"],
        deep_starts=summary["deep_starts"],
        goal_label=html.escape(goal_label),
        goal_value=html.escape(goal_value),
        target_slots=summary["target_slots"],
        wall=summary["wall_time_sec"],
        suite_name=html.escape(suite.get("name", "unknown")),
        commit=html.escape(provenance.get("git_commit", "unknown")),
        binary_sha=html.escape(
            provenance.get("binary_sha256", "unknown")
        ),
        sections="\n".join(sections),
        timeout=summary["timeout_sec"],
        jobs=summary["jobs"],
    )
    return template


def generate_case_animations(
    data, suite_dir, results_dir, output_dir, jobs=8
):
    suite_dir = Path(suite_dir)
    results_dir = Path(results_dir)
    cases_dir = Path(output_dir) / "cases"
    cases_dir.mkdir(parents=True, exist_ok=True)
    solved = [row for row in data["records"] if row["success"]]

    def generate(record):
        name = record["instance"]
        instance = suite_dir / record["manifest"]["yaml"]
        plan = results_dir / "work" / "{}.carrier.plan".format(name)
        if not instance.is_file():
            raise ValueError("missing instance {}".format(instance))
        if not plan.is_file():
            raise ValueError("missing plan {}".format(plan))
        digest = hashlib.sha256(plan.read_bytes()).hexdigest()
        if digest != record["plan_sha256"]:
            raise ValueError(
                "plan SHA mismatch for {}: {} != {}".format(
                    name, digest, record["plan_sha256"]
                )
            )
        output = cases_dir / "{}.html".format(name)
        title = (
            "{} · density {:.1%} · makespan {}".format(
                name,
                record["actual_density"],
                record["final_makespan"],
            )
        )
        process = subprocess.run(
            [
                sys.executable,
                str(VIZ_GENERATOR),
                str(instance),
                str(plan),
                str(output),
                "--title",
                title,
            ],
            capture_output=True,
            text=True,
            timeout=120,
        )
        if process.returncode != 0:
            raise RuntimeError(
                "animation failed for {}: {}".format(
                    name, (process.stdout + process.stderr)[-1000:]
                )
            )
        return output

    completed = []
    with ThreadPoolExecutor(max_workers=jobs) as executor:
        futures = [executor.submit(generate, row) for row in solved]
        for future in as_completed(futures):
            output = future.result()
            completed.append(output)
            print(
                "[{}/{}] {}".format(
                    len(completed), len(solved), output.name
                ),
                flush=True,
            )
    return completed


def write_dashboard(data, output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "index.html").write_text(
        render_dashboard(data), encoding="utf-8"
    )
    payload = {
        "summary": data["summary"],
        "records": data["records"],
        "timing": data["timing"],
    }
    (output_dir / "summary.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    shutil.copyfile(data["rows_path"], output_dir / "rows.csv")
    shutil.copyfile(data["timing_path"], output_dir / "timing.json")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite-dir", type=Path, default=DEFAULT_SUITE)
    parser.add_argument("--results-dir", type=Path, default=DEFAULT_RESULTS)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--jobs", type=int, default=8)
    parser.add_argument("--skip-cases", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    suite_url = os.path.relpath(
        args.suite_dir.resolve(), args.out_dir.resolve()
    ).replace(os.sep, "/")
    data = build_report_data(
        args.results_dir / "rows.csv",
        args.results_dir / "timing.json",
        args.suite_dir / "manifest.json",
        suite_url=suite_url,
    )
    if not args.skip_cases:
        animations = generate_case_animations(
            data,
            args.suite_dir,
            args.results_dir,
            args.out_dir,
            jobs=args.jobs,
        )
        if len(animations) != data["summary"]["solved"]:
            raise RuntimeError("not every solved case received an animation")
    write_dashboard(data, args.out_dir)
    print("dashboard={}".format(args.out_dir / "index.html"))
    print(
        "solved={}/{}".format(
            data["summary"]["solved"], data["summary"]["total"]
        )
    )


if __name__ == "__main__":
    main()
