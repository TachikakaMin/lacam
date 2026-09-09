#!/usr/bin/env python3
"""Generate the data-driven final report for Carrier BR-LaCAM decomposition."""

import argparse
import csv
import html
import json
import os
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from string import Template


PHASE_FIELDS = (
    ("tau", "brd_tau_ms"),
    ("upper BR-LaCAM", "brd_upper_ms"),
    ("task compile", "brd_task_compile_ms"),
    ("matching", "brd_match_ms"),
    ("lower segments", "brd_segment_ms"),
    ("cleanup", "brd_cleanup_ms"),
    ("replay", "brd_replay_ms"),
    ("goal-prefix", "brd_incidental_goal_prefix_ms"),
)


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


def _nonnegative(row, key):
    value = _number(row, key)
    return value if value is not None and value >= 0 else None


def _cost(row):
    makespan = _number(row, "executed_makespan")
    work = _number(row, "weighted_soc")
    if makespan is None or work is None:
        return None
    return (makespan, work)


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
        "solver_time_sum_sec": float(values["solver_time_sum_sec"]),
        "binary_sha256": payload["provenance"]["binary_sha256"],
        "suite_sha256": payload["suite"]["definition_sha256"],
        "tier": payload["suite"].get("tier", ""),
    }


def _compare(baseline_rows, current_rows):
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
    lexicographic = Counter()
    makespan = Counter()
    work = Counter()
    cases = []
    for name in sorted(baseline):
        old_row = baseline[name]
        new_row = current[name]
        old_ok = _success(old_row)
        new_ok = _success(new_row)
        old = _cost(old_row)
        new = _cost(new_row)
        if old_ok and new_ok:
            relation = (
                "better" if new < old
                else "equal" if new == old
                else "worse"
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
        elif new_ok:
            relation = "gained"
        elif old_ok:
            relation = "lost"
        else:
            relation = "both_failed"
        cases.append(
            {
                "instance": name,
                "relation": relation,
                "baseline_cost": list(old) if old is not None else None,
                "current_cost": list(new) if new is not None else None,
            }
        )
    return {
        "total": len(baseline),
        "baseline_solved": len(baseline_solved),
        "current_solved": len(current_solved),
        "common_solved": len(baseline_solved & current_solved),
        "gained": sorted(current_solved - baseline_solved),
        "lost": sorted(baseline_solved - current_solved),
        "both_failed": len(
            set(baseline) - baseline_solved - current_solved
        ),
        "lexicographic": {
            key: lexicographic[key]
            for key in ("better", "equal", "worse")
        },
        "makespan": {
            key: makespan[key]
            for key in ("better", "equal", "worse")
        },
        "work": {
            key: work[key]
            for key in ("better", "equal", "worse")
        },
        "cases": cases,
    }


def _stats(values):
    if not values:
        return {"count": 0, "sum": 0.0, "median": None, "mean": None}
    return {
        "count": len(values),
        "sum": sum(values),
        "median": statistics.median(values),
        "mean": statistics.mean(values),
    }


def _phase_stats(rows):
    output = {}
    scopes = {
        "all": rows,
        "solved": [row for row in rows if _success(row)],
        "failed": [row for row in rows if not _success(row)],
    }
    for label, field in PHASE_FIELDS:
        output[label] = {
            scope: _stats(
                [
                    value
                    for value in (
                        _nonnegative(row, field) for row in scoped_rows
                    )
                    if value is not None
                ]
            )
            for scope, scoped_rows in scopes.items()
        }
    return output


def _failure_counts(rows):
    failures = Counter()
    for row in rows:
        if _success(row):
            continue
        reason = row.get("brd_exit_reason") or row.get("status") or "UNKNOWN"
        failures[reason] += 1
    return dict(sorted(failures.items()))


def _event_summary(rows):
    sums = {}
    for output_key, field in (
        ("match_calls", "brd_match_calls"),
        ("dispatch_epochs", "brd_dispatch_epochs"),
        ("completion_events", "brd_completion_events"),
        ("segments", "brd_segments"),
        ("provisional_reassignments", "brd_provisional_reassignments"),
    ):
        sums[output_key] = int(
            round(
                sum(
                    _nonnegative(row, field) or 0.0
                    for row in rows
                )
            )
        )
    locks = [
        _nonnegative(row, "brd_locked_carriers_max")
        for row in rows
    ]
    sums["max_locked_carriers"] = int(
        max((value for value in locks if value is not None), default=0)
    )
    return sums


def _raw_vs_deliverable(rows):
    solved = [row for row in rows if _success(row)]
    removed = [
        int(round(_nonnegative(row, "brd_goal_prefix_removed") or 0))
        for row in solved
    ]
    raw_ticks = [
        _nonnegative(row, "brd_raw_ticks")
        for row in solved
    ]
    final_ticks = [
        _number(row, "executed_makespan")
        for row in solved
    ]
    return {
        "solved_cases": len(solved),
        "trimmed_cases": sum(value > 0 for value in removed),
        "ticks_removed": sum(removed),
        "max_ticks_removed": max(removed, default=0),
        "raw_ticks_sum": int(
            round(sum(value for value in raw_ticks if value is not None))
        ),
        "deliverable_ticks_sum": int(
            round(sum(value for value in final_ticks if value is not None))
        ),
    }


def _goal_mode_summary(rows, manifest_path):
    manifest = json.loads(
        Path(manifest_path).read_text(encoding="utf-8")
    )
    metadata = {record["id"]: record for record in manifest}
    groups = defaultdict(list)
    for row in rows:
        meta = metadata.get(row["instance"])
        if meta is not None:
            groups[meta["goal_mode"]].append(row)
    return {
        key: {
            "solved": sum(_success(row) for row in grouped_rows),
            "total": len(grouped_rows),
        }
        for key, grouped_rows in sorted(groups.items())
    }


def _historical_summary(rows):
    output = {}
    for method in ("carrier", "carrier_b0", "carrier_b1"):
        method_rows = [row for row in rows if row.get("method") == method]
        solved = [row for row in method_rows if _success(row)]
        output[method] = {
            "solved": len(solved),
            "total": len(method_rows),
            "runtime_sum_sec": sum(
                _number(row, "runtime_sec") or 0.0 for row in method_rows
            ),
            "makespan_sum": int(
                round(
                    sum(
                        _number(row, "executed_makespan") or 0.0
                        for row in solved
                    )
                )
            ),
            "work_sum": int(
                round(
                    sum(
                        _number(row, "weighted_soc") or 0.0
                        for row in solved
                    )
                )
            ),
        }
    return output


def _fmt(value, digits=1):
    if value is None:
        return "—"
    return ("{0:." + str(digits) + "f}").format(value)


def _bwe(values):
    return "{} / {} / {}".format(
        values["better"], values["equal"], values["worse"]
    )


def _relative(path, out_dir):
    return Path(
        os.path.relpath(Path(path).resolve(), Path(out_dir).resolve())
    ).as_posix()


def _phase_rows(phases):
    rows = []
    for label, _ in PHASE_FIELDS:
        phase = phases[label]
        rows.append(
            "<tr><td>{}</td><td>{}</td><td>{}</td><td>{}</td>"
            "<td>{}</td><td>{}</td></tr>".format(
                html.escape(label),
                _fmt(phase["all"]["sum"]),
                _fmt(phase["solved"]["median"], 2),
                _fmt(phase["failed"]["median"], 2),
                _fmt(phase["solved"]["mean"], 2),
                _fmt(phase["failed"]["mean"], 2),
            )
        )
    return "\n".join(rows)


def _failure_rows(failures):
    return "\n".join(
        "<tr><td>{}</td><td>{}</td></tr>".format(
            html.escape(reason), count
        )
        for reason, count in failures.items()
    ) or '<tr><td colspan="2">无失败</td></tr>'


def _goal_rows(goal_modes):
    return "\n".join(
        "<tr><td>{}</td><td>{}/{}</td><td>{:.1f}%</td></tr>".format(
            html.escape(mode),
            values["solved"],
            values["total"],
            (
                100.0 * values["solved"] / values["total"]
                if values["total"]
                else 0.0
            ),
        )
        for mode, values in goal_modes.items()
    )


def _historical_rows(historical):
    labels = {
        "carrier": "carrier",
        "carrier_b0": "carrier_b0",
        "carrier_b1": "carrier_b1",
    }
    return "\n".join(
        "<tr><td>{}</td><td>{}/{}</td><td>{}</td><td>{}</td>"
        "<td>{}s</td></tr>".format(
            labels[method],
            historical[method]["solved"],
            historical[method]["total"],
            historical[method]["makespan_sum"],
            historical[method]["work_sum"],
            _fmt(historical[method]["runtime_sum_sec"]),
        )
        for method in ("carrier", "carrier_b0", "carrier_b1")
    )


def _example_rows(comparison, baseline_prefix, current_prefix):
    ranked = [
        case
        for case in comparison["cases"]
        if case["relation"] in {"better", "worse", "gained", "lost"}
    ]
    priority = {"gained": 0, "lost": 1, "better": 2, "worse": 3}
    ranked.sort(key=lambda case: (priority[case["relation"]], case["instance"]))
    output = []
    for case in ranked[:12]:
        old = case["baseline_cost"]
        new = case["current_cost"]
        old_text = (
            "({}, {})".format(int(old[0]), int(old[1]))
            if old is not None else "—"
        )
        new_text = (
            "({}, {})".format(int(new[0]), int(new[1]))
            if new is not None else "—"
        )
        old_link = (
            '<a href="{}/{}.html">production</a>'.format(
                html.escape(baseline_prefix.rstrip("/"), quote=True),
                html.escape(case["instance"], quote=True),
            )
            if old is not None else "—"
        )
        new_link = (
            '<a href="{}/{}.html">BRD</a>'.format(
                html.escape(current_prefix.rstrip("/"), quote=True),
                html.escape(case["instance"], quote=True),
            )
            if new is not None else "—"
        )
        output.append(
            "<tr><td>{}</td><td>{}</td><td>{} → {}</td>"
            "<td>{} · {}</td></tr>".format(
                html.escape(case["instance"]),
                html.escape(case["relation"]),
                old_text,
                new_text,
                old_link,
                new_link,
            )
        )
    return "\n".join(output) or (
        '<tr><td colspan="4">没有差异样例</td></tr>'
    )


REPORT_TEMPLATE = Template(
    """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Carrier BR-LaCAM 分解基线最终报告</title>
<style>
:root{--bg:#07111d;--panel:#0d1c2b;--line:#29435b;--text:#edf6ff;
--muted:#9fb3c7;--cyan:#37d5ce;--green:#5ddd91;--red:#ff7f88;
--amber:#ffc966}*{box-sizing:border-box}body{margin:0;background:var(--bg);
color:var(--text);font-family:system-ui,-apple-system,"Noto Sans SC",sans-serif;
line-height:1.65}main{max-width:1180px;margin:auto;padding:34px 22px 70px}
h1{font-size:clamp(30px,5vw,52px);line-height:1.18;margin:.25em 0}h2{margin-top:38px}
h3{margin-bottom:8px}p{color:#c5d4e2}a{color:#8bc8ff}.muted{color:var(--muted)}
.cards,.two,.flow{display:grid;gap:14px}.cards{grid-template-columns:
repeat(auto-fit,minmax(190px,1fr));margin:20px 0}.two{grid-template-columns:1fr 1fr}
.flow{grid-template-columns:repeat(6,1fr);align-items:stretch}.card,.panel,.step{
background:linear-gradient(145deg,#11263a,#0a1827);border:1px solid var(--line);
border-radius:14px;padding:18px}.big{font-size:29px;font-weight:800}.good{color:var(--green)}
.bad{color:var(--red)}.warn{border-left:5px solid var(--amber);background:#241d0d;
padding:14px 17px;border-radius:8px;color:#f7dfb5}.step{text-align:center;padding:14px 7px}
.step b{display:block;color:var(--cyan)}.table-wrap{overflow-x:auto;border:1px solid
var(--line);border-radius:12px}table{width:100%;border-collapse:collapse;min-width:650px}
th,td{text-align:left;padding:9px 11px;border-bottom:1px solid #20384e;white-space:nowrap}
th{color:#c8d8e7;background:#102236}.nav{display:flex;flex-wrap:wrap;gap:9px}
.nav a{border:1px solid var(--line);padding:7px 10px;border-radius:8px;text-decoration:none}
code{color:#b8f3ef}.small{font-size:13px}.event{border-top:4px solid var(--cyan)}
@media(max-width:820px){.two{grid-template-columns:1fr}.flow{grid-template-columns:
repeat(2,1fr)}}@media(max-width:500px){.flow{grid-template-columns:1fr}
main{padding:24px 14px}.cards{grid-template-columns:1fr}}
</style>
</head>
<body><main>
<div class="muted small">2026-09-06 · Carrier BR-LaCAM decomposition baseline</div>
<h1>分解流程跑通了，<br>但它还不能替代 production Carrier</h1>
<p>这份报告记录一个研究基线：先用 BR-LaCAM 冻结货架路线，再把路线编译成
task waves；每当任一机器人完成一次 Drop，就立刻重新匹配并再次调用同一个
<code>TAPFPlanner::solve()</code>。实现语义正确，但 full benchmark 显示固定
货架计划与逐段 first-feasible 执行带来明显完备性和质量损失。</p>
<div class="nav">
 <a href="$full_dashboard">BRD full dashboard</a>
 <a href="$comparison_dashboard">production vs BRD</a>
 <a href="$baseline_dashboard">production dashboard</a>
 <a href="$baseline_rows">production rows.csv</a>
 <a href="$current_rows">BRD rows.csv</a>
 <a href="$current_timing">BRD timing.json</a>
 <a href="$approval">review approval</a>
</div>

<section class="cards">
 <div class="card"><div class="big">$headline_solved</div>
  <div class="muted">同一 $total-case full corpus</div></div>
 <div class="card"><div class="big bad">$lex_bwe</div>
  <div class="muted">共同成功 $common 例：严格 (T,W) 更好 / 相同 / 更差</div></div>
 <div class="card"><div class="big">$wall_base s → $wall_current s</div>
  <div class="muted">full wall time；失败更早不等于算法更强</div></div>
 <div class="card"><div class="big">$events</div>
  <div class="muted">completion events；每次都触发重匹配</div></div>
</section>
<div class="warn"><b>最重要的结论：</b>求解率从 $baseline_solved/$total 降到
$current_solved/$total；BRD 新解出 $gained 个 production 失败例，但丢失
$lost 个 production 成功例。它是可审计的分解基线，不是 production 替代品。</div>

<h2>算法流程</h2>
<div class="flow" aria-label="tau to upper to waves to matching to lower to return">
 <div class="step"><b>tau</b>选择目标位置</div>
 <div class="step"><b>upper</b>BR-LaCAM 搬货架</div>
 <div class="step"><b>waves</b>冻结并编译 tasks</div>
 <div class="step"><b>matching</b>全量 Hungarian</div>
 <div class="step"><b>lower</b>同一 TAPFPlanner</div>
 <div class="step"><b>return</b>验证并交付</div>
</div>
<p class="muted">缩写就是：<b>tau → upper → waves → matching → lower → return</b>。
upper 和普通 MAPF 复用同一 LaCAM DFS kernel；lower 不是另一套 planner，而是
从当前完整物理状态再次进入现有 <code>TAPFPlanner::solve()</code>。</p>

<h2>用户纠正后的 completion-event 语义</h2>
<p><b>一次 solve 只到下一个 Drop</b>，不等待一批 task 全部完成。同一拍若有
多个 Drop，一次性记为完成，只触发一轮重匹配。重匹配矩阵是
<b>所有 free robots × 全部 PENDING tasks</b>；已完成 task 移除，
<b>已 Lift 保持 hard lock</b>，直到它自己的 endpoint Drop。</p>
<div class="two">
 <div class="panel event"><h3>R6：未 Lift，立即重匹配</h3>
  <p>R0 先 Drop；R1 还在接近旧任务、尚未 Lift。当前 segment 当场结束，
  R1 的临时 assignment 被释放。R0、R1 与当前 wave 的全部 pending tasks
  重新进入匹配，因此 R1 可以改做更合适的任务。</p></div>
 <div class="panel event"><h3>R7：已 Lift，继续锁定</h3>
  <p>R0 先 Drop；R1 已经 Lift 另一件货架。R1 不进入下一轮匹配，
  robot–task pair 跨 event 保持 hard lock；其他 free robots 和 pending tasks
  正常重新匹配。</p></div>
</div>
<p>实跑累计 $match_calls 次 matching、$dispatch_epochs 个 dispatch epochs、
$segments 个 lower segments；发生 $reassignments 次未 Lift provisional
reassignment，单例最多同时锁定 $max_locks 个 carrying robots。</p>

<h2>同一 full 509 的正面对比</h2>
<div class="table-wrap"><table>
<thead><tr><th>指标</th><th>production Carrier</th><th>Carrier BRD</th></tr></thead>
<tbody>
 <tr><td>solved</td><td>$baseline_solved/$total</td><td>$current_solved/$total</td></tr>
 <tr><td>solver-time sum</td><td>$solver_base s</td><td>$solver_current s</td></tr>
 <tr><td>wall time</td><td>$wall_base s</td><td>$wall_current s</td></tr>
 <tr><td>binary SHA</td><td>$base_binary</td><td>$current_binary</td></tr>
</tbody></table></div>
<p>共同成功 $common 例中，严格词典序 <code>(makespan, work)</code> 为
<b>$lex_bwe</b>；只看 makespan 是 $t_bwe，只看 work 是 $w_bwe。
低 runtime 的主要原因之一是 183 个失败例中许多在上界或逐段搜索阶段提前退出，
不能把它解释为同等质量下更快。</p>

<h3>最明显的结构性差异：goal mode</h3>
<div class="table-wrap"><table>
<thead><tr><th>goal mode</th><th>BRD solved</th><th>成功率</th></tr></thead>
<tbody>$goal_rows</tbody></table></div>
<p>shared-pool 给 upper 更多终点选择；singleton 把每个目标货架固定到唯一终点。
当前分解基线先冻结整条 shelf plan，lower 某一段失败后不会回到 upper 改路线，
所以 singleton 更容易把错误的次序或临时占据固化成 segment timeout。</p>

<h2>时间花在哪里</h2>
<div class="table-wrap"><table>
<thead><tr><th>阶段</th><th>全体累计 ms</th><th>成功 P50</th><th>失败 P50</th>
<th>成功均值</th><th>失败均值</th></tr></thead>
<tbody>$phase_rows</tbody></table></div>
<p>失败例的主要成本集中在 lower segment，而不是 matching。换言之，全量重匹配
本身不是当前最大瓶颈；更大的问题是固定 upper 计划后，逐段物理搜索可能在某个
Drop 前耗尽剩余预算。</p>

<h2>失败原因</h2>
<div class="table-wrap"><table><thead><tr><th>BRD exit reason</th><th>cases</th></tr></thead>
<tbody>$failure_rows</tbody></table></div>

<h2>raw 计划与最终交付计划</h2>
<p>搜索先产生 raw primitive actions，再重放、截到第一个合法原始 goal prefix，
并重新核算成本。$trimmed_cases/$current_solved 个成功例发生截短，共移除
$ticks_removed 拍，最大单例移除 $max_removed 拍。报告中的 makespan/work
全部来自最终验证后的 deliverable plan，而不是 raw 搜索路径。</p>

<h2>差异样例</h2>
<div class="table-wrap"><table>
<thead><tr><th>instance</th><th>结论</th><th>(T,W) production → BRD</th>
<th>动画</th></tr></thead><tbody>$example_rows</tbody></table></div>

<h2>carrier_b0 / carrier_b1：仅作历史背景</h2>
<div class="warn"><b>历史对照（不同 corpus）：</b>下面三列来自另一个
$historical_total-case 旧 benchmark、旧二进制和旧协议，不能与本次 full 509
做严格 head-to-head，也不能据此排序 BRD。</div>
<div class="table-wrap"><table>
<thead><tr><th>method</th><th>solved</th><th>成功例 makespan sum</th>
<th>成功例 work sum</th><th>runtime sum</th></tr></thead>
<tbody>$historical_rows</tbody></table></div>

<h2>验证与边界</h2>
<div class="cards">
 <div class="card"><div class="big">$cpp_tests / $cpp_tests</div>
  <div class="muted">C++ tests</div></div>
 <div class="card"><div class="big">$python_tests / $python_tests</div>
  <div class="muted">Python tests</div></div>
 <div class="card"><div class="big">$compatibility_tests / $compatibility_tests</div>
  <div class="muted">no-pick/place compatibility checks</div></div>
 <div class="card"><div class="big">$review_decision</div>
  <div class="muted">$reviewer_model independent review</div></div>
</div>
<ul>
 <li>已实现：同一 search kernel、冻结 waves、稳定 anonymous shelf identity、
 completion-event 重匹配、Lift 后硬锁、全链路 deadline 与最终 replay。</li>
 <li>尚未实现：segment 失败后返回 upper 改货架路线；对失败 assignment edge
 做约束学习；跨 event 的增量 Hungarian dual/state 复用；upper anytime 改进。</li>
 <li>当前语义是 first-feasible upper + first-completion lower。它保证每个返回
 plan 可重放，不保证分解后仍保持原 production planner 的完备性或质量。</li>
</ul>

<h2>实验协议与证据</h2>
<p class="small muted">同一 suite、seed、validator、objective weights、14 jobs，
每 case 10 秒。Suite SHA: $suite_sha<br>BRD binary SHA: $current_binary<br>
Full corpus approval SHA: $corpus_sha<br>Reviewer: $reviewer_model ·
decision=$review_decision</p>
</main></body></html>"""
)


def generate_report(
    baseline_rows_path,
    baseline_timing_path,
    current_rows_path,
    current_timing_path,
    manifest_path,
    historical_rows_path,
    approval_path,
    out_dir,
    cpp_tests,
    python_tests,
    compatibility_tests,
    baseline_case_prefix="../full_benchmark_rho_v2_20260906/cases",
    current_case_prefix="../full_benchmark_carrier_brd_20260906/cases",
    full_dashboard="../full_benchmark_carrier_brd_20260906/index.html",
    comparison_dashboard=(
        "../full_comparison_rho_v2_vs_carrier_brd_20260906/index.html"
    ),
    baseline_dashboard="../full_benchmark_rho_v2_20260906/index.html",
):
    baseline_rows = _load_rows(baseline_rows_path)
    current_rows = _load_rows(current_rows_path)
    historical_rows = _load_rows(historical_rows_path)
    baseline_timing = _load_timing(baseline_timing_path)
    current_timing = _load_timing(current_timing_path)
    if baseline_timing["suite_sha256"] != current_timing["suite_sha256"]:
        raise ValueError("baseline and current suite hashes differ")
    comparison = _compare(baseline_rows, current_rows)
    phases = _phase_stats(current_rows)
    failures = _failure_counts(current_rows)
    events = _event_summary(current_rows)
    raw = _raw_vs_deliverable(current_rows)
    goal_modes = _goal_mode_summary(current_rows, manifest_path)
    historical = _historical_summary(historical_rows)
    approval = json.loads(
        Path(approval_path).read_text(encoding="utf-8")
    )
    data = {
        "comparison": comparison,
        "timing": {
            "baseline": baseline_timing,
            "current": current_timing,
        },
        "phases": phases,
        "failures": failures,
        "events": events,
        "raw_vs_deliverable": raw,
        "goal_modes": goal_modes,
        "historical": historical,
        "approval": approval,
        "tests": {
            "cpp": int(cpp_tests),
            "python": int(python_tests),
            "compatibility": int(compatibility_tests),
        },
    }
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    historical_total = max(
        (values["total"] for values in historical.values()), default=0
    )
    page = REPORT_TEMPLATE.substitute(
        full_dashboard=html.escape(str(full_dashboard), quote=True),
        comparison_dashboard=html.escape(
            str(comparison_dashboard), quote=True
        ),
        baseline_dashboard=html.escape(
            str(baseline_dashboard), quote=True
        ),
        baseline_rows=html.escape(
            _relative(baseline_rows_path, out_dir), quote=True
        ),
        current_rows=html.escape(
            _relative(current_rows_path, out_dir), quote=True
        ),
        current_timing=html.escape(
            _relative(current_timing_path, out_dir), quote=True
        ),
        approval=html.escape(
            _relative(approval_path, out_dir), quote=True
        ),
        headline_solved=(
            "production Carrier {}/{} → Carrier BRD {}/{}".format(
                comparison["baseline_solved"],
                comparison["total"],
                comparison["current_solved"],
                comparison["total"],
            )
        ),
        total=comparison["total"],
        baseline_solved=comparison["baseline_solved"],
        current_solved=comparison["current_solved"],
        common=comparison["common_solved"],
        gained=len(comparison["gained"]),
        lost=len(comparison["lost"]),
        lex_bwe=_bwe(comparison["lexicographic"]),
        t_bwe=_bwe(comparison["makespan"]),
        w_bwe=_bwe(comparison["work"]),
        wall_base=_fmt(baseline_timing["wall_time_sec"]),
        wall_current=_fmt(current_timing["wall_time_sec"]),
        solver_base=_fmt(baseline_timing["solver_time_sum_sec"]),
        solver_current=_fmt(current_timing["solver_time_sum_sec"]),
        base_binary=html.escape(baseline_timing["binary_sha256"]),
        current_binary=html.escape(current_timing["binary_sha256"]),
        suite_sha=html.escape(current_timing["suite_sha256"]),
        events=events["completion_events"],
        match_calls=events["match_calls"],
        dispatch_epochs=events["dispatch_epochs"],
        segments=events["segments"],
        reassignments=events["provisional_reassignments"],
        max_locks=events["max_locked_carriers"],
        phase_rows=_phase_rows(phases),
        failure_rows=_failure_rows(failures),
        goal_rows=_goal_rows(goal_modes),
        trimmed_cases=raw["trimmed_cases"],
        ticks_removed=raw["ticks_removed"],
        max_removed=raw["max_ticks_removed"],
        example_rows=_example_rows(
            comparison, baseline_case_prefix, current_case_prefix
        ),
        historical_total=historical_total,
        historical_rows=_historical_rows(historical),
        cpp_tests=int(cpp_tests),
        python_tests=int(python_tests),
        compatibility_tests=int(compatibility_tests),
        review_decision=html.escape(str(approval.get("decision", ""))),
        reviewer_model=html.escape(
            str(approval.get("reviewer_model", ""))
        ),
        corpus_sha=html.escape(
            str(approval.get("full_corpus_sha256", ""))
        ),
    )
    (out_dir / "index.html").write_text(page, encoding="utf-8")
    (out_dir / "summary.json").write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-rows", type=Path, required=True)
    parser.add_argument("--baseline-timing", type=Path, required=True)
    parser.add_argument("--current-rows", type=Path, required=True)
    parser.add_argument("--current-timing", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--historical-rows", type=Path, required=True)
    parser.add_argument("--approval", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--cpp-tests", type=int, required=True)
    parser.add_argument("--python-tests", type=int, required=True)
    parser.add_argument("--compatibility-tests", type=int, required=True)
    args = parser.parse_args()
    data = generate_report(
        baseline_rows_path=args.baseline_rows,
        baseline_timing_path=args.baseline_timing,
        current_rows_path=args.current_rows,
        current_timing_path=args.current_timing,
        manifest_path=args.manifest,
        historical_rows_path=args.historical_rows,
        approval_path=args.approval,
        out_dir=args.out_dir,
        cpp_tests=args.cpp_tests,
        python_tests=args.python_tests,
        compatibility_tests=args.compatibility_tests,
    )
    comparison = data["comparison"]
    print(
        "wrote {}: solved {}/{} -> {}/{}; lex {}".format(
            args.out_dir,
            comparison["baseline_solved"],
            comparison["total"],
            comparison["current_solved"],
            comparison["total"],
            _bwe(comparison["lexicographic"]),
        )
    )


if __name__ == "__main__":
    main()
