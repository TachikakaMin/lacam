#!/usr/bin/env python3
"""Generate the final vacancy-aware shared-carrier report."""

import argparse
import csv
import html
import json
import math
from collections import Counter
from decimal import Decimal
from pathlib import Path


def _load_rows(path):
    with Path(path).open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _success(row):
    return str(row.get("success", "")).lower() in {
        "1", "true", "yes"
    }


def _decimal(row, field):
    value = row.get(field)
    if value in ("", None):
        return None
    return Decimal(str(value))


def _cost(row):
    makespan = _decimal(row, "executed_makespan")
    soc = _decimal(row, "weighted_soc")
    if makespan is None or soc is None:
        return None
    return (makespan, soc)


def _json_number(value):
    if value is None:
        return None
    if value == value.to_integral_value():
        return int(value)
    return float(value)


def _display_number(value):
    if value is None:
        return "—"
    if isinstance(value, Decimal):
        return str(_json_number(value))
    if isinstance(value, float) and value.is_integer():
        return str(int(value))
    return str(value)


def _compare_rows(baseline_rows, current_rows):
    baseline = {row["instance"]: row for row in baseline_rows}
    current = {row["instance"]: row for row in current_rows}
    if set(baseline) != set(current):
        raise ValueError("baseline and current case sets differ")

    baseline_solved = {
        name for name, row in baseline.items() if _success(row)
    }
    current_solved = {
        name for name, row in current.items() if _success(row)
    }
    common = baseline_solved & current_solved
    relations = Counter()
    cases = []
    ratios = []
    baseline_makespan_sum = Decimal(0)
    current_makespan_sum = Decimal(0)
    baseline_soc_sum = Decimal(0)
    current_soc_sum = Decimal(0)
    plan_hash_changes = 0

    for name in sorted(baseline):
        old_row = baseline[name]
        new_row = current[name]
        old_cost = _cost(old_row)
        new_cost = _cost(new_row)
        if name in common:
            relation = (
                "better" if new_cost < old_cost
                else "equal" if new_cost == old_cost
                else "worse"
            )
            relations[relation] += 1
            baseline_makespan_sum += old_cost[0]
            current_makespan_sum += new_cost[0]
            baseline_soc_sum += old_cost[1]
            current_soc_sum += new_cost[1]
            if old_cost[0] > 0:
                ratios.append(float(new_cost[0] / old_cost[0]))
            if (
                old_row.get("plan_sha256") and
                new_row.get("plan_sha256") and
                old_row["plan_sha256"] != new_row["plan_sha256"]
            ):
                plan_hash_changes += 1
        elif name in current_solved:
            relation = "gained"
        elif name in baseline_solved:
            relation = "lost"
        else:
            relation = "both_failed"
        cases.append(
            {
                "instance": name,
                "family": new_row.get("family", ""),
                "relation": relation,
                "baseline_cost": (
                    [_json_number(part) for part in old_cost]
                    if old_cost is not None else None
                ),
                "current_cost": (
                    [_json_number(part) for part in new_cost]
                    if new_cost is not None else None
                ),
                "baseline_status": old_row.get("status", ""),
                "current_status": new_row.get("status", ""),
            }
        )

    geometric_ratio = (
        math.exp(sum(math.log(value) for value in ratios) / len(ratios))
        if ratios else None
    )
    return {
        "total": len(baseline),
        "baseline_solved": len(baseline_solved),
        "current_solved": len(current_solved),
        "common_solved": len(common),
        "gained": sorted(current_solved - baseline_solved),
        "lost": sorted(baseline_solved - current_solved),
        "both_failed": len(
            set(baseline) - baseline_solved - current_solved
        ),
        "lexicographic": {
            key: relations[key]
            for key in ("better", "equal", "worse")
        },
        "baseline_makespan_sum": _json_number(baseline_makespan_sum),
        "current_makespan_sum": _json_number(current_makespan_sum),
        "baseline_soc_sum": _json_number(baseline_soc_sum),
        "current_soc_sum": _json_number(current_soc_sum),
        "makespan_geometric_ratio": geometric_ratio,
        "plan_hash_changes": plan_hash_changes,
        "cases": cases,
    }


def _load_timing(path):
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    methods = payload.get("methods", {})
    if len(methods) != 1:
        raise ValueError("timing must contain exactly one method")
    method, values = next(iter(methods.items()))
    return {
        "method": method,
        "wall_time_sec": float(payload["wall_time_sec"]),
        "jobs": int(payload["jobs"]),
        "tasks": int(payload["n_tasks"]),
        "timeout_sec": float(payload["timeout_per_run_sec"]),
        "solved": int(values["solved"]),
        "binary_sha256": payload["provenance"]["binary_sha256"],
        "suite_sha256": payload["suite"]["definition_sha256"],
    }


def _sum_field(rows, field):
    total = Decimal(0)
    for row in rows:
        value = _decimal(row, field)
        if value is not None:
            total += value
    return _json_number(total)


def _brd_summary(phase_rows, current_rows):
    comparison = _compare_rows(phase_rows, current_rows)
    phase = {row["instance"]: row for row in phase_rows}
    current = {row["instance"]: row for row in current_rows}
    semantic_fields = (
        "executed_makespan",
        "weighted_soc",
        "loaded_moves",
        "free_moves",
        "lift_drop",
        "plan_sha256",
    )
    successful_differences = 0
    failure_reason_changes = 0
    for name in sorted(phase):
        old = phase[name]
        new = current[name]
        if _success(old) and _success(new):
            if any(old.get(field) != new.get(field)
                   for field in semantic_fields):
                successful_differences += 1
        elif not _success(old) and not _success(new):
            if old.get("brd_exit_reason") != new.get("brd_exit_reason"):
                failure_reason_changes += 1
    comparison["successful_semantic_differences"] = (
        successful_differences
    )
    comparison["failure_reason_changes"] = failure_reason_changes
    return comparison


def _quick_soc_regressions(comparison):
    regressions = []
    for case in comparison["cases"]:
        old = case["baseline_cost"]
        new = case["current_cost"]
        if old is None or new is None or old[1] == 0:
            continue
        if Decimal(str(new[1])) > Decimal(str(old[1])) * Decimal("1.05"):
            regressions.append(case)
    return regressions


def _cost_text(cost):
    if cost is None:
        return "—"
    return "({}, {})".format(
        _display_number(cost[0]), _display_number(cost[1])
    )


def _case_rows(cases, baseline_prefix, current_prefix, limit=6):
    output = []
    for case in cases[:limit]:
        name = html.escape(case["instance"])
        baseline_link = (
            '<a href="{}/{}.html">rho V2</a>'.format(
                html.escape(baseline_prefix.rstrip("/"), quote=True),
                html.escape(case["instance"], quote=True),
            )
            if case["baseline_cost"] is not None else "—"
        )
        current_link = (
            '<a href="{}/{}.html">当前</a>'.format(
                html.escape(current_prefix.rstrip("/"), quote=True),
                html.escape(case["instance"], quote=True),
            )
            if case["current_cost"] is not None else "—"
        )
        output.append(
            "<tr><td>{}</td><td>{} → {}</td><td>{} · {}</td></tr>".format(
                name,
                _cost_text(case["baseline_cost"]),
                _cost_text(case["current_cost"]),
                baseline_link,
                current_link,
            )
        )
    return "\n".join(output) or (
        '<tr><td colspan="3">没有符合条件的样例</td></tr>'
    )


def _lost_rows(cases, metadata, baseline_prefix):
    output = []
    for case in cases:
        meta = metadata.get(case["instance"], {})
        shape = " / ".join(
            str(meta.get(field, "—"))
            for field in (
                "map_family",
                "density_level",
                "agent_level",
                "task_profile",
                "goal_mode",
            )
        )
        output.append(
            "<tr><td>{}</td><td>{}</td><td>{}</td><td>{}</td></tr>".format(
                html.escape(case["instance"]),
                html.escape(shape),
                _cost_text(case["baseline_cost"]),
                '<a href="{}/{}.html">rho V2 动画</a>'.format(
                    html.escape(baseline_prefix.rstrip("/"), quote=True),
                    html.escape(case["instance"], quote=True),
                ),
            )
        )
    return "\n".join(output) or (
        '<tr><td colspan="4">没有丢解</td></tr>'
    )


def _lost_structure_summary(cases, metadata):
    if not cases:
        return "当前 full 没有新增丢解。"
    records = [metadata.get(case["instance"], {}) for case in cases]
    if not all(
        record.get("map_family") and record.get("goal_mode")
        for record in records
    ):
        return "上表逐例列出 manifest 中可用的结构标签。"
    map_families = {record["map_family"] for record in records}
    goal_modes = {record["goal_mode"] for record in records}
    if len(map_families) != 1 or len(goal_modes) != 1:
        return "这些丢解来自多种结构组合，逐例标签见上表。"
    map_family = next(iter(map_families))
    goal_mode = next(iter(goal_modes)).replace("_", "-")
    scarce_count = sum(
        record.get("agent_level") == "scarce" for record in records
    )
    scarce_text = (
        "，其中 {} 例是 scarce agent".format(scarce_count)
        if scarce_count else ""
    )
    return "这 {} 例都来自 {} 的 {} factorial{}。".format(
        len(cases), map_family, goal_mode, scarce_text
    )


def _quick_regression_rows(cases):
    return "\n".join(
        "<tr><td>{}</td><td>{} → {}</td></tr>".format(
            html.escape(case["instance"]),
            _cost_text(case["baseline_cost"]),
            _cost_text(case["current_cost"]),
        )
        for case in cases
    ) or '<tr><td colspan="2">没有超过 5% 的 SOC 回退</td></tr>'


def generate_report(
    baseline_full_rows,
    baseline_full_timing,
    current_full_rows,
    current_full_timing,
    phase_a_quick_rows,
    current_quick_rows,
    phase_a_brd_full_rows,
    current_brd_full_rows,
    current_brd_full_timing,
    manifest_path,
    approval_path,
    out_dir,
    cpp_tests,
    python_tests,
    full_dashboard="../full_benchmark_vacancy_shared_carrier_20260907/index.html",
    comparison_dashboard=(
        "../full_comparison_rho_v2_vs_vacancy_shared_carrier_20260907/"
        "index.html"
    ),
    baseline_case_prefix="../full_benchmark_rho_v2_20260906/cases",
    current_case_prefix=(
        "../full_benchmark_vacancy_shared_carrier_20260907/cases"
    ),
):
    baseline_full_data = _load_rows(baseline_full_rows)
    current_full_data = _load_rows(current_full_rows)
    phase_quick_data = _load_rows(phase_a_quick_rows)
    current_quick_data = _load_rows(current_quick_rows)
    phase_brd_data = _load_rows(phase_a_brd_full_rows)
    current_brd_data = _load_rows(current_brd_full_rows)
    full = _compare_rows(baseline_full_data, current_full_data)
    quick = _compare_rows(phase_quick_data, current_quick_data)
    brd = _brd_summary(phase_brd_data, current_brd_data)
    quick_regressions = _quick_soc_regressions(quick)
    quick["soc_regressions_over_5pct"] = len(quick_regressions)
    quick["soc_regression_cases"] = [
        case["instance"] for case in quick_regressions
    ]

    baseline_timing = _load_timing(baseline_full_timing)
    current_timing = _load_timing(current_full_timing)
    brd_timing = _load_timing(current_brd_full_timing)
    approval = json.loads(
        Path(approval_path).read_text(encoding="utf-8")
    )
    if approval.get("decision") != "APPROVE":
        raise ValueError("final report requires an APPROVE review")
    if approval.get("blocking_findings") != []:
        raise ValueError("final report requires no blocking findings")
    if approval.get("binary_sha256") != current_timing["binary_sha256"]:
        raise ValueError("approval and current binary hashes differ")
    if approval.get("suite_definition_sha256") != \
            current_timing["suite_sha256"]:
        raise ValueError("approval and current suite hashes differ")
    if brd_timing["binary_sha256"] != current_timing["binary_sha256"]:
        raise ValueError("carrier and carrier_brd binaries differ")

    manifest = json.loads(
        Path(manifest_path).read_text(encoding="utf-8")
    )
    metadata = {record["id"]: record for record in manifest}
    lost_cases = [
        case for case in full["cases"] if case["relation"] == "lost"
    ]
    lost_count = len(lost_cases)
    better_cases = sorted(
        (
            case for case in full["cases"]
            if case["relation"] == "better"
        ),
        key=lambda case: (
            case["baseline_cost"][0] - case["current_cost"][0],
            case["baseline_cost"][1] - case["current_cost"][1],
        ),
        reverse=True,
    )
    worse_cases = sorted(
        (
            case for case in full["cases"]
            if case["relation"] == "worse"
        ),
        key=lambda case: (
            case["current_cost"][0] - case["baseline_cost"][0],
            case["current_cost"][1] - case["baseline_cost"][1],
        ),
        reverse=True,
    )
    telemetry = {
        field: _sum_field(current_full_data, field)
        for field in (
            "vacancy_potential_builds",
            "epoch_first_transfer_flips",
            "upper_epoch_cache_evictions",
        )
    }
    tests = {"cpp": int(cpp_tests), "python": int(python_tests)}

    full_bwe = "{better} / {equal} / {worse}".format(
        **full["lexicographic"]
    )
    full_ratio = (
        100 * (full["makespan_geometric_ratio"] - 1)
        if full["makespan_geometric_ratio"] is not None else 0.0
    )
    brd_preservation = (
        "成功计划零差异"
        if brd["successful_semantic_differences"] == 0
        else "{} 个成功例发生差异".format(
            brd["successful_semantic_differences"]
        )
    )
    page = """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Vacancy-aware shared carrier 最终报告</title>
<style>
:root{{--bg:#111315;--line:#393d41;--text:#f1f3f4;
--muted:#aab0b5;--accent:#79b7e8}}
*{{box-sizing:border-box}}body{{margin:0;background:var(--bg);color:var(--text);
font-family:system-ui,-apple-system,"Noto Sans SC",sans-serif;line-height:1.7;
font-variant-numeric:tabular-nums}}main{{max-width:1080px;margin:auto;
padding:44px 24px 76px}}.report-head{{border-bottom:1px solid var(--line);
padding-bottom:30px}}.meta{{color:var(--muted);font-size:14px}}h1{{font-size:
clamp(38px,6vw,64px);line-height:1.08;margin:12px 0 16px;letter-spacing:-.025em}}
.lede{{font-size:18px;max-width:820px;margin:0;color:#d5d9dc}}h2{{font-size:
26px;line-height:1.25;margin:48px 0 18px}}h3{{font-size:18px;margin:0 0 8px}}
p{{color:#d5d9dc}}a{{color:var(--accent)}}.verdict{{display:grid;
grid-template-columns:minmax(270px,1.15fr) minmax(0,2fr);align-items:stretch;
border-bottom:1px solid var(--line)}}.verdict-main{{padding:26px 28px 24px 0}}
.verdict-label,.measurements dt,.audit-list dt{{color:var(--muted);
font-size:13px}}.verdict-value{{font-size:clamp(25px,3.5vw,38px);
font-weight:750;line-height:1.2;margin-top:7px}}.measurements{{display:grid;
grid-template-columns:repeat(3,1fr);margin:0;border-left:1px solid var(--line)}}
.measurements div{{padding:26px 18px 22px}}.measurements div+div{{border-left:
1px solid var(--line)}}.measurements dd{{font-size:24px;font-weight:700;
margin:6px 0 0}}.reading-note{{margin:24px 0 0;padding:13px 0;border-top:
1px solid var(--line);border-bottom:1px solid var(--line)}}.mechanism-list{{
margin:0}}.mechanism-list div{{display:grid;grid-template-columns:190px 1fr;
gap:24px;padding:12px 0;border-top:1px solid var(--line)}}.mechanism-list
div:last-child{{border-bottom:1px solid var(--line)}}.mechanism-list dt{{
font-weight:700}}.mechanism-list dd{{margin:0;color:#d5d9dc}}.columns{{
display:grid;grid-template-columns:1fr 1fr;gap:48px;margin:0 0 22px}}
.columns section{{min-width:0}}.table-wrap{{overflow-x:auto;border-top:
1px solid var(--line);border-bottom:1px solid var(--line)}}table{{width:100%;
border-collapse:collapse;min-width:680px}}
th,td{{text-align:left;padding:9px 11px;border-bottom:1px solid var(--line);
vertical-align:top}}tbody tr:last-child td{{border-bottom:0}}th{{color:#cbd5dc;
font-weight:650}}code{{color:#b8dcfa}}.audit-list{{margin:0;border-top:
1px solid var(--line)}}.audit-list div{{display:grid;grid-template-columns:
210px 1fr;padding:10px 0;border-bottom:1px solid var(--line)}}.audit-list dd{{
margin:0;font-weight:700}}.links{{columns:2;column-gap:42px;padding-left:20px}}
.links li{{break-inside:avoid;margin:0 0 8px}}.provenance{{border-top:
1px solid var(--line);padding-top:14px;color:var(--muted);font-size:12px;
overflow-wrap:anywhere}}@media(max-width:760px){{main{{padding:28px 15px 60px}}
.verdict,.columns{{grid-template-columns:1fr}}.measurements{{border-left:0;
border-top:1px solid var(--line);grid-template-columns:1fr}}.measurements div{{
padding:14px 0}}.measurements div+div{{border-left:0;border-top:
1px solid var(--line)}}.mechanism-list div{{grid-template-columns:1fr;gap:2px}}
.audit-list div{{grid-template-columns:1fr;gap:2px}}.links{{columns:1}}}}
</style>
</head>
<body><main>
<header class="report-head">
 <div class="meta">2026-09-07 · 同一 509-case 协议 · 14 jobs · 10 秒/例</div>
 <h1>Full 少解 {lost_count} 例</h1>
 <p class="lede">空位引导、跨 epoch 连续性和在途任务恢复已经接入同一条
 LaCAM-TAPF 主路径。固定 quick 的已解集合不变且总成本下降；sealed full
 从 rho V2 的 {base_solved}/{total} 降到 {current_solved}/{total}，其中
 {lost_count} 个新 timeout。机制已经落地，但当前版本还不能替代 rho V2。</p>
</header>

<section class="verdict" aria-label="核心对比">
 <div class="verdict-main">
  <div class="verdict-label">求解率（首要指标）</div>
  <div class="verdict-value">rho V2 {base_solved}/{total} → 当前 {current_solved}/{total}</div>
 </div>
 <dl class="measurements">
  <div><dt>共同成功 {common} 例<br>更好 / 相同 / 更差</dt><dd>{full_bwe}</dd></div>
  <div><dt>共同成功例<br>makespan 几何平均</dt><dd>{full_ratio:+.2f}%</dd></div>
  <div><dt>full wall time<br>rho V2 {base_wall:.1f}s</dt><dd>{wall:.1f}s</dd></div>
 </dl>
</section>
<p class="reading-note"><strong>先看丢解。</strong>共同成功例的成本只描述已解
子集，不能补偿 solved-set 回退，也不用于重新挑选 case 或调参。</p>

<h2>实现边界</h2>
<dl class="mechanism-list">
 <div><dt>空位势能</dt><dd>只给现有合法候选排序。</dd></div>
 <div><dt>有限 continuity</dt><dd>只打破 flexible pool 的 exact-Q tie。</dd></div>
 <div><dt>在途 effect</dt><dd>只恢复无法从当前 frontier 重现的搬运。</dd></div>
 <div><dt>goal commitment</dt><dd>未完成的 flexible root 暂时保留合法目标。</dd></div>
 <div><dt>执行路径</dt><dd>继续进入原 rho、PIBT 与 timed transport。</dd></div>
</dl>
<p>singleton fixed-goal 不继承 continuity；stale source 会被过滤，历史选择
只活一个 epoch。normalization cache 的 key 只有
<code>(upper, raw effects, priority)</code>，没有测试专用维度。无
pick/place 的普通 LaCAM-TAPF 不会生成 carrier task，因此自然保持原行为。</p>

<h2>full 的 {lost_count} 个丢解</h2>
<div class="table-wrap"><table>
<thead><tr><th>实例</th><th>结构</th><th>rho V2 成本</th><th>旧动画</th></tr></thead>
<tbody>{lost_rows}</tbody></table></div>
<p>{lost_structure_summary}
当前行均为 timeout，没有 invalid plan，也没有失败后残留 plan 文件。按照
sealed 协议，这些结果只用于评价，不能反向调整 seed、case 或算法。</p>

<h2>Quick 结果</h2>
<div class="columns">
 <section><h3>Phase A → shared fix</h3>
  <p>公共 {quick_common} 例的 makespan 总和
  <b>{quick_base_mk} → {quick_current_mk}</b>，SOC 总和
  <b>{quick_base_soc} → {quick_current_soc}</b>；solved 为
  {quick_base_solved}/{quick_total} → {quick_current_solved}/{quick_total}。</p>
 </section>
 <section><h3>仍有 {quick_regression_count} 例回退</h3>
  <p>仍有 <b>{quick_regression_count} 个 SOC 超过 5%</b> 的公共成功例。
  quick 是实现前冻结的 77 例，这些回退不会从样本中移除。</p>
 </section>
</div>
<div class="table-wrap"><table>
<thead><tr><th>quick 回退实例</th><th>(T,SOC) Phase A → 当前</th></tr></thead>
<tbody>{quick_regression_rows}</tbody></table></div>

<h2>共同成功例的成本变化</h2>
<div class="columns">
 <section><h3>改善样例</h3><div class="table-wrap"><table>
  <thead><tr><th>实例</th><th>成本</th><th>动画</th></tr></thead>
  <tbody>{better_rows}</tbody></table></div></section>
 <section><h3>回退样例</h3><div class="table-wrap"><table>
  <thead><tr><th>实例</th><th>成本</th><th>动画</th></tr></thead>
  <tbody>{worse_rows}</tbody></table></div></section>
</div>

<h2>carrier_brd 兼容性</h2>
<p>carrier_brd full 为 Phase A {brd_base_solved}/{brd_total} → 当前
{brd_current_solved}/{brd_total}；{brd_preservation}。一个未解例的
exit label 发生变化（共 {brd_reason_changes} 个），但 solved、成功成本、
动作计数和 plan SHA 不变。</p>

<h2>验证与审查</h2>
<dl class="audit-list">
 <div><dt>C++ tests</dt><dd>{cpp_tests} / {cpp_tests}</dd></div>
 <div><dt>Python tests</dt><dd>{python_tests} / {python_tests}</dd></div>
 <div><dt>独立审查</dt><dd>{review_decision} · {reviewer_model} / high</dd></div>
 <div><dt>release binary SHA 前 12 位</dt><dd>{binary_short}</dd></div>
</dl>
<p>reviewer 独立复跑测试并确认：无平行 planner、feature flag、旧算法
fallback、instance/seed hack 或 test-only production API。full gate 同时绑定
reviewer metadata、suite、corpus 与 binary，且 blocking findings 为空。</p>

<h2>原始证据</h2>
<ul class="links">
 <li><a href="{full_dashboard}">当前 full dashboard（{current_solved} 个动画）</a></li>
 <li><a href="{comparison_dashboard}">rho V2 vs 当前逐例对比</a></li>
 <li><a href="evidence/carrier_full/rows.csv">当前 carrier rows.csv</a></li>
 <li><a href="evidence/carrier_brd_full/rows.csv">当前 carrier_brd rows.csv</a></li>
 <li><a href="evidence/carrier_quick/rows.csv">当前 quick rows.csv</a></li>
 <li><a href="evidence/full_review_approval_vacancy_shared_carrier_20260907.json">full_review_approval_vacancy_shared_carrier_20260907.json</a></li>
</ul>
<p class="provenance">binary {binary_sha}<br>suite {suite_sha}<br>
corpus {corpus_sha}<br>telemetry totals: vacancy builds={vacancy_builds},
epoch flips={epoch_flips}, cache evictions={cache_evictions}</p>
</main></body></html>""".format(
        base_solved=full["baseline_solved"],
        current_solved=full["current_solved"],
        total=full["total"],
        lost_count=lost_count,
        full_bwe=full_bwe,
        common=full["common_solved"],
        full_ratio=full_ratio,
        wall=current_timing["wall_time_sec"],
        base_wall=baseline_timing["wall_time_sec"],
        lost_rows=_lost_rows(
            lost_cases, metadata, baseline_case_prefix
        ),
        lost_structure_summary=_lost_structure_summary(
            lost_cases, metadata
        ),
        quick_common=quick["common_solved"],
        quick_base_mk=_display_number(quick["baseline_makespan_sum"]),
        quick_current_mk=_display_number(quick["current_makespan_sum"]),
        quick_base_soc=_display_number(quick["baseline_soc_sum"]),
        quick_current_soc=_display_number(quick["current_soc_sum"]),
        quick_base_solved=quick["baseline_solved"],
        quick_current_solved=quick["current_solved"],
        quick_total=quick["total"],
        quick_regression_count=len(quick_regressions),
        quick_regression_rows=_quick_regression_rows(
            quick_regressions
        ),
        better_rows=_case_rows(
            better_cases, baseline_case_prefix, current_case_prefix
        ),
        worse_rows=_case_rows(
            worse_cases, baseline_case_prefix, current_case_prefix
        ),
        brd_base_solved=brd["baseline_solved"],
        brd_current_solved=brd["current_solved"],
        brd_total=brd["total"],
        brd_preservation=brd_preservation,
        brd_reason_changes=brd["failure_reason_changes"],
        cpp_tests=tests["cpp"],
        python_tests=tests["python"],
        review_decision=html.escape(approval["decision"]),
        reviewer_model=html.escape(approval["reviewer_model"]),
        binary_short=current_timing["binary_sha256"][:12],
        full_dashboard=html.escape(full_dashboard, quote=True),
        comparison_dashboard=html.escape(
            comparison_dashboard, quote=True
        ),
        binary_sha=html.escape(current_timing["binary_sha256"]),
        suite_sha=html.escape(current_timing["suite_sha256"]),
        corpus_sha=html.escape(approval["full_corpus_sha256"]),
        vacancy_builds=telemetry["vacancy_potential_builds"],
        epoch_flips=telemetry["epoch_first_transfer_flips"],
        cache_evictions=telemetry["upper_epoch_cache_evictions"],
    )

    summary = {
        "full": full,
        "quick": quick,
        "brd": brd,
        "timing": {
            "baseline": baseline_timing,
            "current": current_timing,
            "carrier_brd": brd_timing,
        },
        "telemetry": telemetry,
        "tests": tests,
        "approval": approval,
    }
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    evidence_files = {
        "evidence/carrier_full/rows.csv": current_full_rows,
        "evidence/carrier_brd_full/rows.csv": current_brd_full_rows,
        "evidence/carrier_quick/rows.csv": current_quick_rows,
        (
            "evidence/"
            "full_review_approval_vacancy_shared_carrier_20260907.json"
        ): approval_path,
    }
    for relative_path, source_path in evidence_files.items():
        target = out_dir / relative_path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(Path(source_path).read_bytes())
    (out_dir / "index.html").write_text(page, encoding="utf-8")
    (out_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-full-rows", required=True)
    parser.add_argument("--baseline-full-timing", required=True)
    parser.add_argument("--current-full-rows", required=True)
    parser.add_argument("--current-full-timing", required=True)
    parser.add_argument("--phase-a-quick-rows", required=True)
    parser.add_argument("--current-quick-rows", required=True)
    parser.add_argument("--phase-a-brd-full-rows", required=True)
    parser.add_argument("--current-brd-full-rows", required=True)
    parser.add_argument("--current-brd-full-timing", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--approval", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--cpp-tests", type=int, required=True)
    parser.add_argument("--python-tests", type=int, required=True)
    args = parser.parse_args()
    generate_report(
        baseline_full_rows=args.baseline_full_rows,
        baseline_full_timing=args.baseline_full_timing,
        current_full_rows=args.current_full_rows,
        current_full_timing=args.current_full_timing,
        phase_a_quick_rows=args.phase_a_quick_rows,
        current_quick_rows=args.current_quick_rows,
        phase_a_brd_full_rows=args.phase_a_brd_full_rows,
        current_brd_full_rows=args.current_brd_full_rows,
        current_brd_full_timing=args.current_brd_full_timing,
        manifest_path=args.manifest,
        approval_path=args.approval,
        out_dir=args.out_dir,
        cpp_tests=args.cpp_tests,
        python_tests=args.python_tests,
    )


if __name__ == "__main__":
    main()
