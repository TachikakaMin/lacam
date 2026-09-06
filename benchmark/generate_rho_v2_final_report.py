#!/usr/bin/env python3
"""Generate the data-driven final report for the rho V2 dispatch revision."""

import argparse
import csv
import html
import json
import statistics
from pathlib import Path

from generate_full_comparison_dashboard import build_comparison_data


def _load_rows(path):
    with Path(path).open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _success(row):
    return str(row.get("success", "")).lower() in {"1", "true", "yes"}


def _number(row, key):
    value = row.get(key, "")
    return float(value) if value not in ("", None) else None


def _sum_field(rows, key):
    return sum(
        _number(row, key) or 0.0
        for row in rows
        if _success(row)
    )


def _median_first_solution(rows):
    values = [
        _number(row, "first_solution_ms")
        for row in rows
        if _success(row) and _number(row, "first_solution_ms") is not None
    ]
    return statistics.median(values) if values else None


def _find_group(data, axis, key):
    for group in data["groups"].get(axis, []):
        if group["key"] == key:
            return group
    return None


def _cost_pair(pair):
    return "({}, {})".format(pair[0], pair[1])


def _fmt(value, digits=2):
    if value is None:
        return "—"
    return "{:.{digits}f}".format(value, digits=digits)


def _counts(summary):
    return " / ".join(
        str(value)
        for value in summary["lexicographic"]["better_equal_worse"]
    )


def _case_rows(cases, verdict, limit):
    selected = [
        case for case in cases
        if case["verdict"] == verdict
        and case["baseline_makespan"] is not None
        and case["current_makespan"] is not None
    ]
    selected.sort(
        key=lambda case: (
            case["current_makespan"] - case["baseline_makespan"],
            case["current_work"] - case["baseline_work"],
        ),
        reverse=(verdict == "worse"),
    )
    return selected[:limit]


def _case_table(cases, verdict):
    rows = []
    for case in cases:
        rows.append(
            "<tr><td>{}</td><td>{}</td><td>{}</td>"
            "<td><a href=\"{}\">基线</a> · "
            "<a href=\"{}\">V2</a></td></tr>".format(
                html.escape(case["instance"]),
                _cost_pair(
                    (
                        int(case["baseline_makespan"]),
                        int(case["baseline_work"]),
                    )
                ),
                _cost_pair(
                    (
                        int(case["current_makespan"]),
                        int(case["current_work"]),
                    )
                ),
                html.escape(case["baseline_animation"] or "#"),
                html.escape(case["current_animation"] or "#"),
            )
        )
    return "\n".join(rows) or (
        "<tr><td colspan=\"4\">没有 {}</td></tr>".format(
            html.escape(verdict)
        )
    )


def _axis_rows(groups):
    rows = []
    for group in groups:
        better, equal, worse = (
            group["lexicographic"]["better_equal_worse"]
        )
        ratio = group["makespan"]["geometric_ratio"]
        rows.append(
            "<tr><td>{}</td><td>{}</td><td>{} / {} / {}</td>"
            "<td>{:+.2f}%</td></tr>".format(
                html.escape(group["key"]),
                group["common_solved"],
                better,
                equal,
                worse,
                100 * (ratio - 1) if ratio is not None else 0,
            )
        )
    return "\n".join(rows)


def _protected_rows(evaluation):
    rows = []
    for example in evaluation.get("protected_examples", []):
        rows.append(
            "<tr><td>{}</td><td>{}</td><td>{} → {}</td></tr>".format(
                html.escape(example["label"]),
                html.escape(example.get("before_label", "before")),
                _cost_pair(example["before"]),
                _cost_pair(example["after"]),
            )
        )
    return "\n".join(rows)


def _rejected_rows(evaluation):
    rows = []
    for experiment in evaluation.get("rejected_experiments", []):
        rows.append(
            "<tr><td>{}</td><td>{}</td><td>{}</td></tr>".format(
                html.escape(experiment["label"]),
                _cost_pair(experiment["testcase_cost"]),
                html.escape(experiment["reason"]),
            )
        )
    return "\n".join(rows)


def generate_report(
    baseline_rows_path,
    baseline_timing_path,
    current_rows_path,
    current_timing_path,
    manifest_path,
    evaluation_path,
    approval_path,
    out_dir,
    cpp_tests,
    python_tests,
    baseline_case_prefix="../full_benchmark_two_pass_20260906/cases",
    current_case_prefix="../full_benchmark_rho_v2_20260906/cases",
    full_dashboard="../full_benchmark_rho_v2_20260906/index.html",
    comparison_dashboard=(
        "../full_comparison_two_pass_vs_rho_v2_20260906/index.html"
    ),
):
    data = build_comparison_data(
        baseline_rows_path=baseline_rows_path,
        baseline_timing_path=baseline_timing_path,
        current_rows_path=current_rows_path,
        current_timing_path=current_timing_path,
        manifest_path=manifest_path,
        baseline_case_prefix=baseline_case_prefix,
        current_case_prefix=current_case_prefix,
    )
    baseline_rows = _load_rows(baseline_rows_path)
    current_rows = _load_rows(current_rows_path)
    evaluation = json.loads(
        Path(evaluation_path).read_text(encoding="utf-8")
    )
    approval = json.loads(
        Path(approval_path).read_text(encoding="utf-8")
    )
    objective_versions = sorted(
        {
            row["rho_objective_version"]
            for row in current_rows
            if _success(row) and row.get("rho_objective_version")
        }
    )
    telemetry = {
        "objective_versions": objective_versions,
        "candidates_input": int(
            _sum_field(current_rows, "rho_candidates_input")
        ),
        "candidates_after_priority": int(
            _sum_field(current_rows, "rho_candidates_after_priority")
        ),
        "priority_filtered": int(
            _sum_field(current_rows, "rho_priority_filtered")
        ),
        "matrix_rows": int(
            _sum_field(current_rows, "rho_matrix_rows_total")
        ),
        "assignment_changes": int(
            _sum_field(current_rows, "rho_assignment_changes")
        ),
        "guidance_time_ms": _sum_field(
            current_rows, "guidance_time_ms"
        ),
    }
    data["tests"] = {
        "cpp": int(cpp_tests),
        "python": int(python_tests),
    }
    data["evaluation"] = evaluation
    data["approval"] = approval
    data["telemetry"] = telemetry
    data["first_solution"] = {
        "baseline_median_ms": _median_first_solution(baseline_rows),
        "current_median_ms": _median_first_solution(current_rows),
    }

    overview = data["overview"]
    factorial = data["factorial"]
    quick = data["quick"]
    scarce = _find_group(data, "agent_level", "scarce")
    equal = _find_group(data, "agent_level", "equal")
    surplus = _find_group(data, "agent_level", "surplus")
    better_cases = _case_rows(data["cases"], "better", 6)
    worse_cases = _case_rows(data["cases"], "worse", 6)
    known_issues = "".join(
        "<li>{}</li>".format(html.escape(issue))
        for issue in evaluation.get("known_issues", [])
    )
    objective_name = (
        objective_versions[0] if len(objective_versions) == 1
        else ", ".join(objective_versions) or "未导出"
    )

    page = """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Carrier-LaCAM rho V2 最终汇报</title>
<style>
:root{{--bg:#07111d;--panel:#0d1b2a;--line:#20364d;--text:#e8f0f8;
--muted:#9eb0c2;--cyan:#50d6d7;--green:#54d68c;--red:#ff7b7b;
--amber:#ffc96b}}*{{box-sizing:border-box}}body{{margin:0;background:var(--bg);
color:var(--text);font-family:system-ui,-apple-system,"Noto Sans SC",sans-serif;
line-height:1.65}}main{{max-width:1180px;margin:auto;padding:40px 24px 72px}}
h1{{font-size:clamp(30px,5vw,54px);line-height:1.15;margin:.2em 0}}
h2{{margin-top:44px}}a{{color:var(--cyan)}}.lead{{font-size:19px;color:#c9d8e6;
max-width:920px}}.warning{{border-left:5px solid var(--amber);padding:16px 20px;
background:#251d0e;border-radius:8px}}.cards{{display:grid;
grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:14px;margin:24px 0}}
.card,.panel{{background:var(--panel);border:1px solid var(--line);
border-radius:14px;padding:20px}}.big{{font-size:31px;font-weight:750}}
.sub{{color:var(--muted);font-size:14px}}.good{{color:var(--green)}}
.bad{{color:var(--red)}}.flow{{display:grid;grid-template-columns:1fr auto 1fr;
gap:14px;align-items:center}}.flow pre{{white-space:pre-wrap;margin:0;
font:15px/1.6 ui-monospace,SFMono-Regular,monospace}}.arrow{{font-size:30px;
color:var(--cyan)}}table{{width:100%;border-collapse:collapse;margin:14px 0}}
th,td{{text-align:left;padding:10px 12px;border-bottom:1px solid var(--line)}}
th{{color:#bfd0df}}.cols{{display:grid;grid-template-columns:1fr 1fr;gap:18px}}
.tag{{display:inline-block;border:1px solid var(--line);border-radius:999px;
padding:3px 9px;color:var(--muted);font-size:13px}}code{{color:#b7f1f2}}
@media(max-width:760px){{.cols,.flow{{grid-template-columns:1fr}}.arrow{{transform:rotate(90deg);
text-align:center}}table{{font-size:13px}}th,td{{padding:8px 6px}}}}
</style>
</head>
<body><main>
<span class="tag">2026-09-06 · production {production_commit}</span>
<h1>候选准入修好了，<br>但 full 结果不是整体质量胜利</h1>
<p class="lead">这次修改让普通低 priority 任务真正进入全局匹配，并用严格
direct-target gate 限制有限延期权重。合法性和求解率保持，但 {total} 例显示：
新规则在一部分实例显著改善，也在更多实例上变差，尤其是机器人稀缺场景。</p>

<div class="cards">
 <div class="card"><div class="big">{current_solved}/{total}</div>
  <div class="sub">solved，与基线 {baseline_solved}/{total} 完全相同</div></div>
 <div class="card"><div class="big">{full_bwe}</div>
  <div class="sub">full 词典序：更好 / 相同 / 更差</div></div>
 <div class="card"><div class="big bad">{factorial_ratio:+.2f}%</div>
  <div class="sub">{factorial_total} factorial 的几何平均 makespan 变化</div></div>
 <div class="card"><div class="big">{wall_current:.1f}s</div>
  <div class="sub">full wall time；基线 {wall_baseline:.1f}s</div></div>
</div>

<div class="warning"><b>结论边界：</b>V2 可以作为“候选公平性与直接交付连续性”
的正确实现，但不能宣称它让整体 makespan 更好。full 中 {full_better} 个实例更好、
{full_worse} 个更差；factorial 中 {factorial_bwe}。后续研究应优先处理 scarce-agent
下的 bottleneck 调度，不应把 additive 或增量 Hungarian 当作自动解法。</div>

<h2>改了什么</h2>
<div class="flow">
 <div class="panel"><b>审计前</b><pre>ready tasks
→ priority top-F 硬过滤
→ 只有幸存任务进入矩阵
→ bottleneck + secondary</pre></div>
 <div class="arrow">→</div>
 <div class="panel"><b>当前 V2</b><pre>物理/因果可行 tasks
→ 全部进入 task-row 矩阵
→ 混合清障：原物理 bottleneck
→ 纯直接交付：有限 frontier / continuity defer
→ 无 INF、无 mandatory、无 priority cutoff</pre></div>
</div>
<p>成功行导出的 objective 是 <code>{objective}</code>；累计
<code>priority_filtered={priority_filtered}</code>，且
<code>after_priority={after_priority:,}</code> 与
<code>matrix_rows={matrix_rows:,}</code> 相同。</p>

<h2>full {total}：先看整体，再看结构</h2>
<div class="cards">
 <div class="card"><div class="big">{quick_bwe}</div><div class="sub">
  quick {quick_common} 个共同成功实例：更好 / 相同 / 更差</div></div>
 <div class="card"><div class="big">{factorial_bwe}</div><div class="sub">
  factorial {factorial_common} 例：更好 / 相同 / 更差</div></div>
 <div class="card"><div class="big">{first_base:.0f} → {first_current:.0f} ms</div>
  <div class="sub">成功实例首解时间中位数</div></div>
 <div class="card"><div class="big">{assignment_changes:,}</div>
  <div class="sub">相邻 guidance assignment-id changes；不是增量 repair 次数</div></div>
</div>
<table><thead><tr><th>机器人供给</th><th>共同成功</th>
<th>更好 / 相同 / 更差</th><th>几何平均 makespan</th></tr></thead>
<tbody>{agent_rows}</tbody></table>
<p>最清楚的信号来自 <b>scarce</b>：{scarce_bwe}，几何平均 makespan
{scarce_ratio:+.2f}%。相反，equal 和 surplus 基本保持不变
（{equal_bwe}；{surplus_bwe}）。说明 V2 的直接交付延期在机器人不足时更容易
牺牲全局 bottleneck，而机器人够用时通常不改变选择。</p>

<div class="cols">
 <section class="panel"><h3>改善最大的例子</h3>
  <table><thead><tr><th>实例</th><th>基线</th><th>V2</th><th>动画</th></tr></thead>
  <tbody>{better_rows}</tbody></table></section>
 <section class="panel"><h3>回退最大的例子</h3>
  <table><thead><tr><th>实例</th><th>基线</th><th>V2</th><th>动画</th></tr></thead>
  <tbody>{worse_rows}</tbody></table></section>
</div>

<h2>为什么没有发布 additive / incremental Hungarian</h2>
<p>候选模型决定“派谁做什么”，增量求解只决定“多快算出同一个答案”。
实验先实现 additive full solver，再实现节点局部 shadow；结果如下：</p>
<table><thead><tr><th>实验</th><th>保护实例成本</th><th>决策</th></tr></thead>
<tbody>{rejected_rows}</tbody></table>
<p>因此 G/H 被完整回滚。当前 production 没有 additive path、node-local
matching state 或运行时切换开关。</p>

<h2>保护样例与验证</h2>
<table><thead><tr><th>样例</th><th>对照</th><th>成本变化</th></tr></thead>
<tbody>{protected_rows}</tbody></table>
<div class="cards">
 <div class="card"><div class="big">{cpp_tests} / {cpp_tests}</div>
  <div class="sub">C++ tests</div></div>
 <div class="card"><div class="big">{python_tests} / {python_tests}</div>
  <div class="sub">Python tests</div></div>
 <div class="card"><div class="big">{review_decision}</div>
  <div class="sub">独立 {reviewer_model} / high 终审</div></div>
</div>

<h2>已知限制</h2>
<ul>{known_issues}</ul>
<p>另外，baseline→V2 有 {hash_changes} 个共同成功实例改变了计划 SHA，
这是算法行为变化的预期结果；同一 V2 的 quick/full 字节差异见上方已知限制。</p>

<h2>查看原始证据</h2>
<p><a href="{full_dashboard}">V2 full dashboard（{current_solved} 个真实动画）</a> ·
<a href="{comparison_dashboard}">逐 case 基线对照 dashboard</a> ·
<a href="../../results_full_rho_v2_20260906/rows.csv">current rows.csv</a> ·
<a href="../../results_full_two_pass_reference_20260906/rows.csv">baseline rows.csv</a></p>
<p class="sub">binary <code>{binary_sha}</code><br>
suite <code>{suite_sha}</code><br>
approval: {review_decision} · {reviewer_model}</p>
</main></body></html>""".format(
        production_commit=html.escape(
            evaluation.get("production_commit", "")
        ),
        current_solved=overview["current_solved"],
        baseline_solved=overview["baseline_solved"],
        total=overview["total"],
        full_bwe=_counts(overview),
        full_better=overview["lexicographic"]["better_equal_worse"][0],
        full_worse=overview["lexicographic"]["better_equal_worse"][2],
        factorial_ratio=100 * (
            factorial["makespan"]["geometric_ratio"] - 1
        ),
        factorial_total=factorial["total"],
        wall_current=data["timing"]["current"]["wall_time_sec"],
        wall_baseline=data["timing"]["baseline"]["wall_time_sec"],
        objective=html.escape(objective_name),
        priority_filtered=telemetry["priority_filtered"],
        after_priority=telemetry["candidates_after_priority"],
        matrix_rows=telemetry["matrix_rows"],
        quick_bwe=_counts(quick),
        quick_common=quick["common_solved"],
        factorial_bwe=_counts(factorial),
        factorial_common=factorial["common_solved"],
        first_base=data["first_solution"]["baseline_median_ms"],
        first_current=data["first_solution"]["current_median_ms"],
        assignment_changes=telemetry["assignment_changes"],
        agent_rows=_axis_rows(data["groups"]["agent_level"]),
        scarce_bwe=_counts(scarce) if scarce else "—",
        scarce_ratio=(
            100 * (scarce["makespan"]["geometric_ratio"] - 1)
            if scarce else 0
        ),
        equal_bwe=_counts(equal) if equal else "—",
        surplus_bwe=_counts(surplus) if surplus else "—",
        better_rows=_case_table(better_cases, "better"),
        worse_rows=_case_table(worse_cases, "worse"),
        rejected_rows=_rejected_rows(evaluation),
        protected_rows=_protected_rows(evaluation),
        cpp_tests=int(cpp_tests),
        python_tests=int(python_tests),
        review_decision=html.escape(approval.get("decision", "")),
        reviewer_model=html.escape(approval.get("reviewer_model", "")),
        known_issues=known_issues,
        hash_changes=overview["plan_hash_changes"],
        full_dashboard=html.escape(full_dashboard),
        comparison_dashboard=html.escape(comparison_dashboard),
        binary_sha=html.escape(
            data["timing"]["current"]["binary_sha256"]
        ),
        suite_sha=html.escape(
            data["timing"]["current"]["suite_sha256"]
        ),
    )

    summary = {
        key: data[key]
        for key in (
            "overview",
            "quick",
            "factorial",
            "timing",
            "groups",
            "tests",
            "evaluation",
            "approval",
            "telemetry",
            "first_solution",
        )
    }
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "index.html").write_text(page, encoding="utf-8")
    (out_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return data


def main():
    parser = argparse.ArgumentParser(
        description="Generate the final rho V2 benchmark report."
    )
    parser.add_argument("--baseline-full-rows", required=True)
    parser.add_argument("--baseline-full-timing", required=True)
    parser.add_argument("--current-full-rows", required=True)
    parser.add_argument("--current-full-timing", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--evaluation", required=True)
    parser.add_argument("--approval", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--cpp-tests", type=int, required=True)
    parser.add_argument("--python-tests", type=int, required=True)
    args = parser.parse_args()
    data = generate_report(
        baseline_rows_path=args.baseline_full_rows,
        baseline_timing_path=args.baseline_full_timing,
        current_rows_path=args.current_full_rows,
        current_timing_path=args.current_full_timing,
        manifest_path=args.manifest,
        evaluation_path=args.evaluation,
        approval_path=args.approval,
        out_dir=args.out_dir,
        cpp_tests=args.cpp_tests,
        python_tests=args.python_tests,
    )
    print(
        "wrote {}: {}/{} solved; lex {}".format(
            args.out_dir,
            data["overview"]["current_solved"],
            data["overview"]["total"],
            _counts(data["overview"]),
        )
    )


if __name__ == "__main__":
    main()
