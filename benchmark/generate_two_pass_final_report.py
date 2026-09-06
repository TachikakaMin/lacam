#!/usr/bin/env python3
"""Generate the data-driven final report for the two-pass Carrier-LaCAM."""

import argparse
import csv
import html
import json
import os
import statistics
from collections import Counter
from pathlib import Path


def _load_rows(path):
    with Path(path).open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _success(row):
    return str(row.get("success", "")).lower() in {
        "1", "true", "yes"
    }


def _number(row, key):
    value = row.get(key)
    if value in ("", None):
        return None
    return float(value)


def _integer(row, key):
    value = _number(row, key)
    return None if value is None else int(round(value))


def _cost(row, prefix=""):
    makespan = _number(row, prefix + "makespan")
    work = _number(row, prefix + "soc")
    if prefix == "":
        makespan = _number(row, "executed_makespan")
        work = _number(row, "weighted_soc")
    if makespan is None or work is None:
        return None
    return (makespan, work)


def _runtime_ms(row):
    value = _number(row, "solver_runtime_ms")
    if value is not None:
        return value
    seconds = _number(row, "runtime_sec")
    return None if seconds is None else seconds * 1000


def _percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * fraction))
    return ordered[index]


def _timing_stats(values):
    return {
        "mean": statistics.mean(values) if values else None,
        "median": statistics.median(values) if values else None,
        "p90": _percentile(values, 0.9),
        "max": max(values) if values else None,
    }


def _planning_time_summary(rows, timeout_sec):
    pairs = []
    for row in rows:
        if not _success(row):
            continue
        first_ms = _number(row, "first_solution_ms")
        final_ms = _runtime_ms(row)
        if (
            first_ms is None
            or final_ms is None
            or first_ms < 0
            or final_ms < 0
        ):
            continue
        pairs.append((first_ms, final_ms))

    first_times = [pair[0] for pair in pairs]
    final_times = [pair[1] for pair in pairs]
    delays = [max(0, final - first) for first, final in pairs]
    timeout_ms = max(1.0, float(timeout_sec) * 1000.0)
    horizon_ms = max(
        [timeout_ms] + final_times if final_times else [timeout_ms]
    )
    cdf = []
    for index in range(101):
        elapsed_ms = horizon_ms * index / 100
        cdf.append(
            {
                "ms": elapsed_ms,
                "first_count": sum(
                    value <= elapsed_ms for value in first_times
                ),
                "final_count": sum(
                    value <= elapsed_ms for value in final_times
                ),
            }
        )
    milestones = {}
    for elapsed_ms in (1000, 2000, 5000, 8000, 9000):
        if elapsed_ms > horizon_ms:
            continue
        milestones[str(elapsed_ms)] = {
            "first_count": sum(
                value <= elapsed_ms for value in first_times
            ),
            "final_count": sum(
                value <= elapsed_ms for value in final_times
            ),
        }
    return {
        "paired_count": len(pairs),
        "horizon_ms": horizon_ms,
        "first_ms": _timing_stats(first_times),
        "final_ms": _timing_stats(final_times),
        "delay_ms": _timing_stats(delays),
        "cdf": cdf,
        "milestones": milestones,
    }


def _summary(rows):
    solved = [row for row in rows if _success(row)]
    runtimes = [
        value for value in (_runtime_ms(row) for row in rows)
        if value is not None
    ]
    first_times = [
        value
        for value in (
            _number(row, "first_solution_ms") for row in solved
        )
        if value is not None and value >= 0
    ]
    makespans = [
        value
        for value in (
            _number(row, "executed_makespan") for row in solved
        )
        if value is not None
    ]
    work = [
        value
        for value in (
            _number(row, "weighted_soc") for row in solved
        )
        if value is not None
    ]
    first_to_final = {"better": 0, "equal": 0, "worse": 0}
    for row in solved:
        first = _cost(row, "first_solution_")
        final = _cost(row)
        if first is None or final is None:
            continue
        if final < first:
            first_to_final["better"] += 1
        elif final == first:
            first_to_final["equal"] += 1
        else:
            first_to_final["worse"] += 1
    return {
        "total": len(rows),
        "solved": len(solved),
        "failed": len(rows) - len(solved),
        "success_rate": len(solved) / len(rows) if rows else 0,
        "makespan_sum": sum(makespans),
        "work_sum": sum(work),
        "runtime_ms_mean": (
            statistics.mean(runtimes) if runtimes else None
        ),
        "runtime_ms_median": (
            statistics.median(runtimes) if runtimes else None
        ),
        "runtime_ms_max": max(runtimes) if runtimes else None,
        "first_solution_ms_mean": (
            statistics.mean(first_times) if first_times else None
        ),
        "first_solution_ms_median": (
            statistics.median(first_times) if first_times else None
        ),
        "first_solution_ms_p90": _percentile(first_times, 0.9),
        "first_solution_count": len(first_times),
        "first_to_final": first_to_final,
    }


def _compare(baseline_rows, current_rows):
    baseline = {row["instance"]: row for row in baseline_rows}
    current = {row["instance"]: row for row in current_rows}
    baseline_solved = {
        name for name, row in baseline.items() if _success(row)
    }
    current_solved = {
        name for name, row in current.items() if _success(row)
    }
    common = sorted(baseline_solved & current_solved)
    lexicographic = {"better": 0, "equal": 0, "worse": 0}
    makespan = {"better": 0, "equal": 0, "worse": 0}
    work = {"better": 0, "equal": 0, "worse": 0}
    examples = {"better": [], "worse": []}
    for name in common:
        old = _cost(baseline[name])
        new = _cost(current[name])
        if old is None or new is None:
            continue
        relation = (
            "better" if new < old else "equal" if new == old else "worse"
        )
        lexicographic[relation] += 1
        makespan[
            "better" if new[0] < old[0]
            else "equal" if new[0] == old[0]
            else "worse"
        ] += 1
        work[
            "better" if new[1] < old[1]
            else "equal" if new[1] == old[1]
            else "worse"
        ] += 1
        if relation in examples and len(examples[relation]) < 5:
            examples[relation].append(
                {
                    "instance": name,
                    "baseline": [old[0], old[1]],
                    "current": [new[0], new[1]],
                }
            )
    return {
        "common_solved": len(common),
        "lexicographic": lexicographic,
        "makespan": makespan,
        "work": work,
        "gained": sorted(current_solved - baseline_solved),
        "lost": sorted(baseline_solved - current_solved),
        "success_sets_equal": baseline_solved == current_solved,
        "examples": examples,
    }


def _phase2_diagnostics(rows):
    exits = Counter(
        row.get("improvement_exit_reason", "")
        for row in rows
        if _success(row) and row.get("improvement_exit_reason", "")
    )
    reference_keys = {
        "checkpoint_hits": "reference_checkpoint_hits",
        "action_hints": "reference_action_hints",
        "suffix_attempts": "reference_suffix_attempts",
        "suffix_accepted": "reference_suffix_accepted",
    }
    reference = {
        label: sum(
            _integer(row, key) or 0 for row in rows
        )
        for label, key in reference_keys.items()
    }
    suffix_rows = [
        row for row in rows
        if (_integer(row, "reference_suffix_accepted") or 0) > 0
    ]
    suffix_example = None
    if suffix_rows:
        row = suffix_rows[0]
        first = _cost(row, "first_solution_")
        final = _cost(row)
        suffix_example = {
            "instance": row["instance"],
            "first": (
                [first[0], first[1]] if first is not None else None
            ),
            "final": (
                [final[0], final[1]] if final is not None else None
            ),
            "runtime_ms": _runtime_ms(row),
        }
    return {
        "phase2_exit_reasons": dict(sorted(exits.items())),
        "reference": reference,
        "suffix_example": suffix_example,
    }


def _load_timing(path):
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    return {
        "wall_time_sec": float(payload["wall_time_sec"]),
        "jobs": int(payload["jobs"]),
        "tasks": int(payload["n_tasks"]),
        "timeout_sec": float(payload["timeout_per_run_sec"]),
        "solver_time_sum_sec": float(
            payload["methods"]["carrier"]["solver_time_sum_sec"]
        ),
        "binary_sha256": payload["provenance"]["binary_sha256"],
        "execution_snapshot": payload["provenance"].get(
            "execution_snapshot", ""
        ),
        "suite_sha256": payload["suite"]["definition_sha256"],
        "tier": payload["suite"].get("tier", ""),
    }


def _suite_data(
    baseline_rows, current_rows, baseline_timing, current_timing
):
    output = {
        "baseline": _summary(baseline_rows),
        "current": _summary(current_rows),
        "comparison": _compare(baseline_rows, current_rows),
        "timing": {
            "baseline": baseline_timing,
            "current": current_timing,
        },
        "planning_time": _planning_time_summary(
            current_rows, current_timing["timeout_sec"]
        ),
    }
    output.update(_phase2_diagnostics(current_rows))
    return output


def build_report_data(
    baseline_quick_rows_path,
    current_quick_rows_path,
    baseline_quick_timing_path,
    current_quick_timing_path,
    baseline_full_rows_path,
    current_full_rows_path,
    baseline_full_timing_path,
    current_full_timing_path,
    historical_full_rows_path,
    historical_full_timing_path,
    approval_path,
    cpp_tests,
    python_tests,
):
    baseline_quick_rows = _load_rows(baseline_quick_rows_path)
    current_quick_rows = _load_rows(current_quick_rows_path)
    baseline_full_rows = _load_rows(baseline_full_rows_path)
    current_full_rows = _load_rows(current_full_rows_path)
    historical_full_rows = _load_rows(historical_full_rows_path)
    full_rows = (
        baseline_full_rows,
        current_full_rows,
        historical_full_rows,
    )
    full_case_sets = [
        {row["instance"] for row in rows} for rows in full_rows
    ]
    if any(
        len(case_set) != len(rows)
        for case_set, rows in zip(full_case_sets, full_rows)
    ):
        raise ValueError("full report inputs contain duplicate instances")
    if not all(case_set == full_case_sets[0] for case_set in full_case_sets):
        raise ValueError("full report inputs use different case sets")

    baseline_quick_timing = _load_timing(baseline_quick_timing_path)
    current_quick_timing = _load_timing(current_quick_timing_path)
    baseline_full_timing = _load_timing(baseline_full_timing_path)
    current_full_timing = _load_timing(current_full_timing_path)
    historical_full_timing = _load_timing(historical_full_timing_path)
    for timing in (
        baseline_full_timing,
        historical_full_timing,
    ):
        if (
            timing["tasks"] != current_full_timing["tasks"]
            or timing["jobs"] != current_full_timing["jobs"]
            or timing["timeout_sec"] != current_full_timing["timeout_sec"]
            or timing["suite_sha256"]
            != current_full_timing["suite_sha256"]
        ):
            raise ValueError("full report timing protocols differ")

    approval = json.loads(
        Path(approval_path).read_text(encoding="utf-8")
    )
    data = {
        "quick": _suite_data(
            baseline_quick_rows,
            current_quick_rows,
            baseline_quick_timing,
            current_quick_timing,
        ),
        "full": _suite_data(
            baseline_full_rows,
            current_full_rows,
            baseline_full_timing,
            current_full_timing,
        ),
        "historical_full": _suite_data(
            historical_full_rows,
            current_full_rows,
            historical_full_timing,
            current_full_timing,
        ),
        "tests": {
            "cpp": int(cpp_tests),
            "python": int(python_tests),
        },
        "approval": approval,
    }
    current_hash = data["full"]["timing"]["current"]["binary_sha256"]
    if approval.get("schema_version") != 2:
        raise ValueError("unexpected full benchmark approval schema")
    if approval.get("decision") != "APPROVE":
        raise ValueError("full benchmark approval is not APPROVE")
    if approval.get("reviewer_model") != "openai.gpt-5.6-sol":
        raise ValueError("unexpected full benchmark reviewer model")
    if approval.get("binary_sha256") != current_hash:
        raise ValueError("approval and full result binary hashes differ")
    if (
        approval.get("suite_definition_sha256")
        != current_full_timing["suite_sha256"]
    ):
        raise ValueError("approval and full result suite hashes differ")
    return data


def _fmt(value, digits=1):
    if value is None:
        return "—"
    return ("{0:." + str(digits) + "f}").format(value)


def _cost_text(cost):
    if cost is None:
        return "—"
    return "({}, {})".format(
        int(round(cost[0])), int(round(cost[1]))
    )


def _bwe(metric):
    return "{}/{}/{}".format(
        metric["better"], metric["equal"], metric["worse"]
    )


def _exit_rows(exits):
    labels = {
        "STRICT_IMPROVEMENT": "直接找到第一份严格改进",
        "REFERENCE_SUFFIX_ACCEPTED": "新前缀接上首轮合法后缀",
        "SEARCH_CUTOFF": "未找到改进，截止后回退首轮",
        "SEARCH_EXHAUSTED": "固定目标空间搜索完毕",
        "NO_SECOND_ATTEMPT": "剩余预算不足，未启动第二遍",
    }
    return "\n".join(
        "<tr><td><code>{}</code></td><td>{}</td><td>{}</td></tr>".format(
            html.escape(reason),
            count,
            html.escape(labels.get(reason, "其他明确退出原因")),
        )
        for reason, count in sorted(exits.items())
    ) or '<tr><td colspan="3">没有记录第二遍退出原因</td></tr>'


def _examples(rows):
    output = []
    for kind, label in (("better", "改进"), ("worse", "变差")):
        for row in rows.get(kind, []):
            output.append(
                "<tr><td>{}</td><td>{}</td><td>{}</td><td>{}</td></tr>".format(
                    html.escape(row["instance"]),
                    label,
                    _cost_text(row["baseline"]),
                    _cost_text(row["current"]),
                )
            )
    return "\n".join(output) or (
        '<tr><td colspan="4">共同求解案例没有成本差异</td></tr>'
    )


def _duration_text(milliseconds):
    if milliseconds is None:
        return "—"
    if milliseconds < 1000:
        return "{} ms".format(int(round(milliseconds)))
    return "{:.2f} s".format(milliseconds / 1000)


def _timing_chart(planning_time):
    count = planning_time["paired_count"]
    points = planning_time["cdf"]
    if count <= 0 or not points:
        return '<p class="muted">没有可配对的首解与最终返回时间。</p>'

    width, height = 1000, 430
    left, right, top, bottom = 76, 28, 28, 62
    plot_width = width - left - right
    plot_height = height - top - bottom
    horizon_ms = planning_time["horizon_ms"]

    def x_position(milliseconds):
        return left + plot_width * milliseconds / horizon_ms

    def y_position(case_count):
        return top + plot_height * (1 - case_count / count)

    first_points = " ".join(
        "{:.1f},{:.1f}".format(
            x_position(point["ms"]),
            y_position(point["first_count"]),
        )
        for point in points
    )
    final_points = " ".join(
        "{:.1f},{:.1f}".format(
            x_position(point["ms"]),
            y_position(point["final_count"]),
        )
        for point in points
    )

    grid = []
    for percentage in (0, 25, 50, 75, 100):
        y = top + plot_height * (1 - percentage / 100)
        grid.append(
            '<line x1="{left}" y1="{y:.1f}" x2="{right}" '
            'y2="{y:.1f}" class="chart-grid"/>'
            '<text x="{label_x}" y="{label_y:.1f}" '
            'class="chart-label" text-anchor="end">{value}%</text>'.format(
                left=left,
                right=width - right,
                y=y,
                label_x=left - 12,
                label_y=y + 4,
                value=percentage,
            )
        )
    for index in range(6):
        elapsed_ms = horizon_ms * index / 5
        x = x_position(elapsed_ms)
        grid.append(
            '<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" '
            'y2="{bottom}" class="chart-grid"/>'
            '<text x="{x:.1f}" y="{label_y}" class="chart-label" '
            'text-anchor="middle">{seconds:.0f}s</text>'.format(
                x=x,
                top=top,
                bottom=height - bottom,
                label_y=height - bottom + 25,
                seconds=elapsed_ms / 1000,
            )
        )

    first_median = planning_time["first_ms"]["median"]
    final_median = planning_time["final_ms"]["median"]
    median_y = y_position(count / 2)
    markers = []
    for value, color, label in (
        (first_median, "#2463eb", "首解 P50"),
        (final_median, "#14865a", "最终返回 P50"),
    ):
        if value is None:
            continue
        x = x_position(value)
        markers.append(
            '<circle cx="{x:.1f}" cy="{y:.1f}" r="6" '
            'fill="{color}" stroke="white" stroke-width="2">'
            '<title>{label}: {duration}</title></circle>'.format(
                x=x,
                y=median_y,
                color=color,
                label=label,
                duration=_duration_text(value),
            )
        )

    return """
<svg class="timing-chart" viewBox="0 0 {width} {height}"
     role="img" aria-labelledby="timing-chart-title timing-chart-desc">
  <title id="timing-chart-title">首解可用时间与最终返回时间累计分布</title>
  <desc id="timing-chart-desc">横轴为规划耗时，纵轴为累计案例比例。</desc>
  {grid}
  <polyline points="{first_points}" class="timing-line first-line"/>
  <polyline points="{final_points}" class="timing-line final-line"/>
  {markers}
  <text x="{x_label}" y="{x_label_y}" class="chart-axis-title"
        text-anchor="middle">规划耗时</text>
  <text x="18" y="{y_label}" class="chart-axis-title"
        text-anchor="middle" transform="rotate(-90 18 {y_label})">
        累计案例比例
  </text>
  <g transform="translate({legend_x},18)">
    <line x1="0" y1="0" x2="32" y2="0" class="timing-line first-line"/>
    <text x="41" y="5" class="chart-label">首解可用</text>
    <line x1="150" y1="0" x2="182" y2="0" class="timing-line final-line"/>
    <text x="191" y="5" class="chart-label">最终返回</text>
  </g>
</svg>
""".format(
        width=width,
        height=height,
        grid="\n  ".join(grid),
        first_points=first_points,
        final_points=final_points,
        markers="\n  ".join(markers),
        x_label=left + plot_width / 2,
        x_label_y=height - 8,
        y_label=top + plot_height / 2,
        legend_x=width - 380,
    )


def _generate_html(data, links):
    quick = data["quick"]
    full = data["full"]
    historical = data["historical_full"]
    qcmp = quick["comparison"]
    fcmp = full["comparison"]
    hcmp = historical["comparison"]
    fs = full["current"]
    planning_time = full["planning_time"]
    timing_chart = _timing_chart(planning_time)
    one_second = planning_time["milestones"].get(
        "1000", {"first_count": 0, "final_count": 0}
    )
    eight_seconds = planning_time["milestones"].get(
        "8000", {"first_count": 0, "final_count": 0}
    )
    suffix = full["suffix_example"]
    suffix_html = (
        (
            "<p><b>{instance}</b>：首轮成本 {first}，拼接并验证后为 "
            "{final}，求解器运行 {runtime} ms。这是“完整物理状态相同”"
            "时复用 primitive-action 后缀的真实案例。</p>"
        ).format(
            instance=html.escape(suffix["instance"]),
            first=_cost_text(suffix["first"]),
            final=_cost_text(suffix["final"]),
            runtime=_fmt(suffix["runtime_ms"], 0),
        )
        if suffix is not None
        else "<p>本次 full benchmark 没有触发可接受的后缀拼接。</p>"
    )
    return """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Carrier-LaCAM 两遍规划最终报告</title>
<style>
:root{{--ink:#172033;--muted:#647086;--line:#dbe2ec;--paper:#f6f8fc;
--blue:#2463eb;--green:#14865a;--amber:#b36a00;--red:#b93838}}
*{{box-sizing:border-box}} body{{margin:0;background:var(--paper);color:var(--ink);
font:16px/1.65 system-ui,-apple-system,"Noto Sans SC","Microsoft YaHei",sans-serif}}
main{{max-width:1120px;margin:auto;padding:36px 22px 70px}} h1{{font-size:38px;
line-height:1.2;margin:0 0 10px}} h2{{margin-top:42px;font-size:25px}}
h3{{margin:0 0 8px}} .lead{{font-size:19px;color:#3b465b;max-width:900px}}
.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:14px}}
.card{{background:white;border:1px solid var(--line);border-radius:14px;padding:18px;
box-shadow:0 4px 18px #18233a0b}} .big{{font-size:30px;font-weight:750}}
.muted{{color:var(--muted)}} .flow{{display:grid;grid-template-columns:repeat(4,1fr);
gap:10px;align-items:stretch}} .step{{background:white;border:1px solid var(--line);
border-top:4px solid var(--blue);border-radius:12px;padding:14px}}
.step:nth-child(2){{border-top-color:var(--green)}} .step:nth-child(3){{
border-top-color:var(--amber)}} .step:nth-child(4){{border-top-color:#7650c5}}
table{{width:100%;border-collapse:collapse;background:white;border-radius:12px;
overflow:hidden}} th,td{{padding:10px 12px;text-align:left;border-bottom:1px solid var(--line)}}
th{{background:#edf2fa}} code{{background:#eef2f8;padding:2px 5px;border-radius:5px}}
.table-wrap{{max-width:100%;overflow-x:auto;border-radius:12px}}
td,th,code,a,footer{{overflow-wrap:anywhere}}
.formula{{font:600 18px ui-monospace,SFMono-Regular,monospace;background:#172033;
color:white;padding:15px 18px;border-radius:10px;overflow:auto}}
.note{{border-left:4px solid var(--amber);background:#fff8e8;padding:12px 15px}}
.good{{color:var(--green)}} .bad{{color:var(--red)}} a{{color:var(--blue)}}
.chart-panel{{margin-top:16px;background:white;border:1px solid var(--line);
border-radius:14px;padding:14px;overflow:hidden}}
.timing-chart{{display:block;width:100%;height:auto;min-height:300px}}
.chart-grid{{stroke:#dfe6f0;stroke-width:1}}
.chart-label{{fill:#647086;font:13px system-ui,-apple-system,"Noto Sans SC",sans-serif}}
.chart-axis-title{{fill:#3b465b;font:600 14px system-ui,-apple-system,
"Noto Sans SC",sans-serif}}
.timing-line{{fill:none;stroke-width:4;stroke-linecap:round;stroke-linejoin:round}}
.first-line{{stroke:var(--blue)}} .final-line{{stroke:var(--green)}}
footer{{margin-top:42px;color:var(--muted);font-size:14px}}
@media(max-width:760px){{.flow{{grid-template-columns:1fr}}h1{{font-size:31px}}}}
</style>
</head>
<body><main>
<h1>Carrier-LaCAM：两遍规划控制器</h1>
<p class="lead">最终实现保留同一个 Carrier-LaCAM 搜索内核。第一遍动态选择
货架目标并尽快拿到可交付首解；第二遍从同一个初态重新搜索，固定首轮终态，
只寻找第一份严格改进。找不到改进就返回首轮。</p>

<div class="grid">
  <div class="card"><div class="muted">full 求解</div><div class="big">{fsolved}/{ftotal}</div></div>
  <div class="card"><div class="muted">full 成本 改进/相同/变差</div><div class="big">{fbwe}</div></div>
  <div class="card"><div class="muted">首解中位时间</div><div class="big">{first_median} ms</div></div>
  <div class="card"><div class="muted">参考后缀接受</div><div class="big">{suffix_count}</div></div>
</div>

<h2>算法从输入到返回</h2>
<div class="flow">
  <div class="step"><h3>1. 第一遍</h3><b>第一遍：先拿到可交付首解</b>。使用完整
  goal sets，停止于第一份可行计划。</div>
  <div class="step"><h3>2. 快照</h3>规范化、repair、完整重放并保存 primitive
  actions、精确成本和目标货架实际终态。</div>
  <div class="step"><h3>3. 第二遍</h3><b>第二遍：固定终态，只找第一份严格改进</b>。
  首轮成本是上界，admissible 下界负责安全剪枝。</div>
  <div class="step"><h3>4. 返回</h3>新候选完整验证且严格更好就立刻返回；
  <b>找不到改进就返回首轮</b>，整体仍是 solved。</div>
</div>

<h2>唯一的成本顺序</h2>
<p><b>makespan 优先，work 次优先</b>；规划 runtime 只受 deadline 约束，不参与
计划质量比较。</p>
<div class="formula">J(P) = (T, W)；接受 P₂ 当且仅当
T₂ &lt; T₁，或 T₂ = T₁ 且 W₂ &lt; W₁</div>
<p>因此 (50, 110) 可以优于 (54, 90)，而 (54, 90) 只是打平，第二遍不会接受。</p>

<h2>benchmark 总览</h2>
<div class="table-wrap"><table>
<thead><tr><th>层级</th><th>当前求解</th><th>共同求解成本 B/E/W</th>
<th>成功集合</th><th>wall time</th><th>单例上限</th></tr></thead>
<tbody>
<tr><td>quick</td><td>{qsolved}/{qtotal}</td><td>{qbwe}</td>
<td>{qsets}</td><td>{qwall}s</td><td>{qtimeout}s</td></tr>
<tr><td>full</td><td>{fsolved}/{ftotal}</td><td>{fbwe}</td>
<td>{fsets}</td><td>{fwall}s</td><td>{ftimeout}s</td></tr>
</tbody></table></div>
<p class="muted">B/E/W 表示相对上一版在严格词典序成本上的
better / equal / worse。变差行是停止策略从 anytime 改为“第一份严格改进即返”
的可见代价，不应隐藏。</p>

<h3>两套历史 full 基线</h3>
<div class="table-wrap"><table>
<thead><tr><th>比较</th><th>基线求解</th><th>当前求解</th>
<th>成本 B/E/W</th><th>基线首解中位</th><th>当前首解中位</th>
<th>基线 runtime 平均</th><th>当前 runtime 平均</th></tr></thead>
<tbody>
<tr><td>旧 v5 anytime full</td>
<td>{old_full_solved}/{ftotal}</td><td>{fsolved}/{ftotal}</td>
<td>{fbwe}</td><td>{old_first_median} ms</td><td>{first_median} ms</td>
<td>{old_runtime_mean} ms</td><td>{current_runtime_mean} ms</td></tr>
<tr><td>pre-v5 full</td>
<td>{historical_solved}/{ftotal}</td><td>{fsolved}/{ftotal}</td>
<td>{historical_bwe}</td><td>{historical_first_median} ms</td>
<td>{first_median} ms</td><td>{historical_runtime_mean} ms</td>
<td>{current_runtime_mean} ms</td></tr>
</tbody></table></div>
<p class="muted">旧 v5 是第二遍继续 anytime 的直接对照；pre-v5 基本不做
当前两遍改进，因此更快，但计划质量和当前 makespan-first 目标不同。三者使用
相同 full case 集合和 10 秒协议。</p>

<h2>首解可用时间 vs 最终返回时间</h2>
<div class="grid">
  <div class="card"><div class="muted">配对案例</div>
    <div class="big">{paired_count}</div></div>
  <div class="card"><div class="muted">首解 P50</div>
    <div class="big">{first_median_text}</div></div>
  <div class="card"><div class="muted">最终返回 P50</div>
    <div class="big">{final_median_text}</div></div>
  <div class="card"><div class="muted">首解 P90</div>
    <div class="big">{first_p90_text}</div></div>
  <div class="card"><div class="muted">最终返回 P90</div>
    <div class="big">{final_p90_text}</div></div>
  <div class="card"><div class="muted">首解到返回 P50</div>
    <div class="big">{delay_median_text}</div></div>
</div>
<div class="chart-panel">{timing_chart}</div>
<p class="note">曲线越靠左，表示越早覆盖更多案例。1 秒时已有
<b>{one_second_first}/{paired_count}</b> 个案例拿到首解，但只有
<b>{one_second_final}/{paired_count}</b> 个完成最终返回；8 秒时分别为
<b>{eight_seconds_first}/{paired_count}</b> 和
<b>{eight_seconds_final}/{paired_count}</b>。</p>
<p>首解时间回答“多久第一次有保底计划”；最终返回时间使用
<code>solver_runtime_ms</code>，包含第二遍、验证、repair 和清理。它们都是规划
runtime，不是计划的执行 makespan。最终交付计划相对首解的质量
改进/相同/异常变差为 <b>{first_final}</b>。</p>

<h2>第二遍如何结束</h2>
<div class="table-wrap"><table><thead><tr><th>退出原因</th><th>数量</th><th>含义</th></tr></thead>
<tbody>{exit_rows}</tbody></table></div>
<p class="note"><b>共享同一个 10 秒 deadline。</b>旧实现曾在 adapter 先预留约
1.5 秒后，又在 kernel 内重复预留约 0.85 秒，所以常在约 7.65 秒停止扩展。
现在只保留外层一次交付余量。找到改进会提前返回；找不到改进的案例仍可用剩余
搜索时间，因此接近 9 秒并不代表第三遍或继续 anytime。</p>

<h2>首轮路径如何帮助第二轮</h2>
<p>参考动作只调整合法候选的尝试顺序。只有机器人位置、两类货架位置和 custody
都完全相同，才会尝试“新前缀 + 首轮 primitive-action 后缀”。<b>参考后缀不是下界</b>：
接旧后缀不够好，不能据此剪掉可能存在更短新后缀的节点。</p>
{suffix_html}
<div class="table-wrap"><table><thead><tr><th>checkpoint 命中</th><th>动作提示</th>
<th>后缀尝试</th><th>后缀接受</th></tr></thead><tbody><tr>
<td>{hits}</td><td>{hints}</td><td>{attempts}</td><td>{suffix_count}</td>
</tr></tbody></table></div>

<h2>代表性变化与负结果</h2>
<div class="table-wrap"><table><thead><tr><th>案例</th><th>结果</th><th>上一版 (T,W)</th>
<th>当前 (T,W)</th></tr></thead><tbody>{examples}</tbody></table></div>
<p>固定 goal 会缩小终态集合，但不会自动消除中间物理状态；如果没有快速证明
“不可能严格改进”的下界，第二遍仍可能运行到 cutoff。对固定 assignment 的
无改进证明，也不等于原始多 goal 问题全局最优。</p>

<h2>验证与可追溯性</h2>
<p>本次通过 <b>{cpp_tests} 个 C++ 测试</b>和 <b>{python_tests} 个 Python 测试</b>。
full benchmark 由独立 GPT-5.6 Sol/high reviewer 审批，二进制、suite 和完整 corpus
哈希均绑定到审批文件。</p>
<ul>
  <li><a href="{approval_link}">独立审批 JSON</a></li>
  <li><a href="{quick_rows_link}">quick 原始 rows.csv</a></li>
  <li><a href="{baseline_full_rows_link}">旧 v5 full rows.csv</a></li>
  <li><a href="{baseline_full_timing_link}">旧 v5 full timing.json</a></li>
  <li><a href="{full_rows_link}">当前 full rows.csv</a></li>
  <li><a href="{full_timing_link}">当前 full timing.json</a></li>
  <li><a href="{historical_rows_link}">pre-v5 full 原始 rows.csv</a></li>
  <li><a href="{historical_timing_link}">pre-v5 full timing.json</a></li>
</ul>
<footer>binary SHA-256: <code>{binary}</code><br>
suite SHA-256: <code>{suite}</code><br>
review model: <code>{reviewer}</code></footer>
</main></body></html>
""".format(
        fsolved=fs["solved"],
        ftotal=fs["total"],
        fbwe=_bwe(fcmp["lexicographic"]),
        suffix_count=full["reference"]["suffix_accepted"],
        first_median=_fmt(fs["first_solution_ms_median"], 0),
        old_full_solved=full["baseline"]["solved"],
        old_first_median=_fmt(
            full["baseline"]["first_solution_ms_median"], 0
        ),
        old_runtime_mean=_fmt(
            full["baseline"]["runtime_ms_mean"], 0
        ),
        current_runtime_mean=_fmt(fs["runtime_ms_mean"], 0),
        historical_solved=historical["baseline"]["solved"],
        historical_bwe=_bwe(hcmp["lexicographic"]),
        historical_first_median=_fmt(
            historical["baseline"]["first_solution_ms_median"], 0
        ),
        historical_runtime_mean=_fmt(
            historical["baseline"]["runtime_ms_mean"], 0
        ),
        qsolved=quick["current"]["solved"],
        qtotal=quick["current"]["total"],
        qbwe=_bwe(qcmp["lexicographic"]),
        qsets="相同" if qcmp["success_sets_equal"] else "有变化",
        qwall=_fmt(quick["timing"]["current"]["wall_time_sec"], 1),
        qtimeout=_fmt(quick["timing"]["current"]["timeout_sec"], 0),
        fsets="相同" if fcmp["success_sets_equal"] else "有变化",
        fwall=_fmt(full["timing"]["current"]["wall_time_sec"], 1),
        ftimeout=_fmt(full["timing"]["current"]["timeout_sec"], 0),
        first_count=fs["first_solution_count"],
        first_mean=_fmt(fs["first_solution_ms_mean"], 0),
        first_p90=_fmt(fs["first_solution_ms_p90"], 0),
        paired_count=planning_time["paired_count"],
        first_median_text=_duration_text(
            planning_time["first_ms"]["median"]
        ),
        final_median_text=_duration_text(
            planning_time["final_ms"]["median"]
        ),
        first_p90_text=_duration_text(
            planning_time["first_ms"]["p90"]
        ),
        final_p90_text=_duration_text(
            planning_time["final_ms"]["p90"]
        ),
        delay_median_text=_duration_text(
            planning_time["delay_ms"]["median"]
        ),
        timing_chart=timing_chart,
        one_second_first=one_second["first_count"],
        one_second_final=one_second["final_count"],
        eight_seconds_first=eight_seconds["first_count"],
        eight_seconds_final=eight_seconds["final_count"],
        first_final=_bwe(fs["first_to_final"]),
        exit_rows=_exit_rows(full["phase2_exit_reasons"]),
        suffix_html=suffix_html,
        hits=full["reference"]["checkpoint_hits"],
        hints=full["reference"]["action_hints"],
        attempts=full["reference"]["suffix_attempts"],
        examples=_examples(fcmp["examples"]),
        cpp_tests=data["tests"]["cpp"],
        python_tests=data["tests"]["python"],
        approval_link=html.escape(links["approval"]),
        quick_rows_link=html.escape(links["quick_rows"]),
        baseline_full_rows_link=html.escape(
            links["baseline_full_rows"]
        ),
        baseline_full_timing_link=html.escape(
            links["baseline_full_timing"]
        ),
        full_rows_link=html.escape(links["full_rows"]),
        full_timing_link=html.escape(links["full_timing"]),
        historical_rows_link=html.escape(links["historical_rows"]),
        historical_timing_link=html.escape(
            links["historical_timing"]
        ),
        binary=html.escape(
            full["timing"]["current"]["binary_sha256"]
        ),
        suite=html.escape(
            full["timing"]["current"]["suite_sha256"]
        ),
        reviewer=html.escape(data["approval"]["reviewer_model"]),
    )


def generate_report(
    baseline_quick_rows_path,
    current_quick_rows_path,
    baseline_quick_timing_path,
    current_quick_timing_path,
    baseline_full_rows_path,
    current_full_rows_path,
    baseline_full_timing_path,
    current_full_timing_path,
    historical_full_rows_path,
    historical_full_timing_path,
    approval_path,
    out_dir,
    cpp_tests,
    python_tests,
):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    data = build_report_data(
        baseline_quick_rows_path,
        current_quick_rows_path,
        baseline_quick_timing_path,
        current_quick_timing_path,
        baseline_full_rows_path,
        current_full_rows_path,
        baseline_full_timing_path,
        current_full_timing_path,
        historical_full_rows_path,
        historical_full_timing_path,
        approval_path,
        cpp_tests,
        python_tests,
    )
    links = {
        "approval": os.path.relpath(
            Path(approval_path).resolve(), out_dir.resolve()
        ),
        "quick_rows": os.path.relpath(
            Path(current_quick_rows_path).resolve(), out_dir.resolve()
        ),
        "baseline_full_rows": os.path.relpath(
            Path(baseline_full_rows_path).resolve(), out_dir.resolve()
        ),
        "baseline_full_timing": os.path.relpath(
            Path(baseline_full_timing_path).resolve(),
            out_dir.resolve(),
        ),
        "full_rows": os.path.relpath(
            Path(current_full_rows_path).resolve(), out_dir.resolve()
        ),
        "full_timing": os.path.relpath(
            Path(current_full_timing_path).resolve(), out_dir.resolve()
        ),
        "historical_rows": os.path.relpath(
            Path(historical_full_rows_path).resolve(),
            out_dir.resolve(),
        ),
        "historical_timing": os.path.relpath(
            Path(historical_full_timing_path).resolve(),
            out_dir.resolve(),
        ),
    }
    (out_dir / "summary.json").write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (out_dir / "index.html").write_text(
        _generate_html(data, links),
        encoding="utf-8",
    )
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-quick-rows", required=True)
    parser.add_argument("--current-quick-rows", required=True)
    parser.add_argument("--baseline-quick-timing", required=True)
    parser.add_argument("--current-quick-timing", required=True)
    parser.add_argument("--baseline-full-rows", required=True)
    parser.add_argument("--current-full-rows", required=True)
    parser.add_argument("--baseline-full-timing", required=True)
    parser.add_argument("--current-full-timing", required=True)
    parser.add_argument("--historical-full-rows", required=True)
    parser.add_argument("--historical-full-timing", required=True)
    parser.add_argument("--approval", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--cpp-tests", required=True, type=int)
    parser.add_argument("--python-tests", required=True, type=int)
    args = parser.parse_args()
    data = generate_report(
        baseline_quick_rows_path=args.baseline_quick_rows,
        current_quick_rows_path=args.current_quick_rows,
        baseline_quick_timing_path=args.baseline_quick_timing,
        current_quick_timing_path=args.current_quick_timing,
        baseline_full_rows_path=args.baseline_full_rows,
        current_full_rows_path=args.current_full_rows,
        baseline_full_timing_path=args.baseline_full_timing,
        current_full_timing_path=args.current_full_timing,
        historical_full_rows_path=args.historical_full_rows,
        historical_full_timing_path=args.historical_full_timing,
        approval_path=args.approval,
        out_dir=args.out_dir,
        cpp_tests=args.cpp_tests,
        python_tests=args.python_tests,
    )
    print(
        "report={} full={}/{}".format(
            Path(args.out_dir) / "index.html",
            data["full"]["current"]["solved"],
            data["full"]["current"]["total"],
        )
    )


if __name__ == "__main__":
    main()
