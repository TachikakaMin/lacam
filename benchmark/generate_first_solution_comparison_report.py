#!/usr/bin/env python3
"""Build a self-contained first-solution comparison report for Carrier and BRD.

The report intentionally keeps the historical full-509 result separate from
the current full-518 experiment.  It uses only ``first_solution_ms`` for the
method comparison; no final/deliverable runtime is substituted for either
side.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
from pathlib import Path


TIMEOUT_MS = 10_000.0
PLOT_FLOOR_MS = 0.5


def read_rows(path: Path) -> dict[str, dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as source:
        rows = {row["instance"]: row for row in csv.DictReader(source)}
    if not rows:
        raise ValueError(f"no rows in {path}")
    return rows


def read_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as source:
        return json.load(source)


def solved(row: dict[str, str] | None) -> bool:
    return row is not None and row.get("success") == "1"


def first_ms(row: dict[str, str] | None) -> float | None:
    if not solved(row):
        return None
    raw = row.get("first_solution_ms", "")
    if raw in ("", None):
        raise ValueError(f"solved case has no first_solution_ms: {row['instance']}")
    return float(raw)


def one_decimal(value: float | None) -> str:
    if value is None:
        return "—"
    if value == int(value):
        return f"{int(value):,}"
    return f"{value:,.1f}"


def percent(value: float | None) -> str:
    return "—" if value is None else f"{value * 100:.1f}%"


def geometric_mean(values: list[float]) -> float | None:
    if not values:
        return None
    return math.exp(sum(math.log(value) for value in values) / len(values))


def comparison_summary(
    carrier: dict[str, dict[str, str]],
    brd: dict[str, dict[str, str]],
    names: list[str],
) -> dict:
    counts = {
        "total": len(names),
        "carrier": 0,
        "brd": 0,
        "both": 0,
        "carrier_only": 0,
        "brd_only": 0,
        "neither": 0,
        "carrier_faster": 0,
        "equal": 0,
        "carrier_slower": 0,
    }
    ratios: list[float] = []
    carrier_sum = 0.0
    brd_sum = 0.0
    for name in names:
        left = carrier.get(name)
        right = brd.get(name)
        left_ok = solved(left)
        right_ok = solved(right)
        counts["carrier"] += int(left_ok)
        counts["brd"] += int(right_ok)
        if left_ok and right_ok:
            counts["both"] += 1
            left_ms = first_ms(left)
            right_ms = first_ms(right)
            assert left_ms is not None and right_ms is not None
            carrier_sum += left_ms
            brd_sum += right_ms
            # A zero/zero trivial instance has no meaningful multiplicative
            # speed ratio.  It remains an equal first solution in counts, but
            # is excluded from geometric/median ratio statistics.
            if left_ms != 0 or right_ms != 0:
                ratios.append(
                    max(left_ms, PLOT_FLOOR_MS) / max(right_ms, PLOT_FLOOR_MS)
                )
            if left_ms < right_ms:
                counts["carrier_faster"] += 1
            elif left_ms > right_ms:
                counts["carrier_slower"] += 1
            else:
                counts["equal"] += 1
        elif left_ok:
            counts["carrier_only"] += 1
        elif right_ok:
            counts["brd_only"] += 1
        else:
            counts["neither"] += 1
    return {
        **counts,
        "carrier_first_sum_ms": carrier_sum,
        "brd_first_sum_ms": brd_sum,
        "ratio_cases": len(ratios),
        "carrier_over_brd_geomean": geometric_mean(ratios),
        "carrier_over_brd_median": (
            sorted(ratios)[len(ratios) // 2] if len(ratios) % 2 else
            (sorted(ratios)[len(ratios) // 2 - 1] + sorted(ratios)[len(ratios) // 2]) / 2
        ) if ratios else None,
    }


def longitudinal_summary(
    old: dict[str, dict[str, str]],
    current: dict[str, dict[str, str]],
    names: list[str],
) -> dict:
    paired = [
        name for name in names
        if solved(old.get(name)) and solved(current.get(name))
    ]
    old_times = [first_ms(old[name]) for name in paired]
    new_times = [first_ms(current[name]) for name in paired]
    assert all(value is not None for value in old_times + new_times)
    old_f = [float(value) for value in old_times if value is not None]
    new_f = [float(value) for value in new_times if value is not None]
    ratios = [
        max(new_ms, PLOT_FLOOR_MS) / max(old_ms, PLOT_FLOOR_MS)
        for old_ms, new_ms in zip(old_f, new_f)
        if old_ms != 0 or new_ms != 0
    ]
    return {
        "paired": len(paired),
        "old_sum_ms": sum(old_f),
        "new_sum_ms": sum(new_f),
        "ratio_cases": len(ratios),
        "new_over_old_geomean": geometric_mean(ratios),
        "new_faster": sum(new_ms < old_ms for old_ms, new_ms in zip(old_f, new_f)),
        "equal": sum(new_ms == old_ms for old_ms, new_ms in zip(old_f, new_f)),
        "new_slower": sum(new_ms > old_ms for old_ms, new_ms in zip(old_f, new_f)),
    }


def group_for(carrier_row: dict[str, str] | None, brd_row: dict[str, str] | None) -> str:
    carrier_ok = solved(carrier_row)
    brd_ok = solved(brd_row)
    if carrier_ok and brd_ok:
        left = first_ms(carrier_row)
        right = first_ms(brd_row)
        assert left is not None and right is not None
        if left < right:
            return "carrier_faster"
        if left > right:
            return "brd_faster"
        return "equal"
    if carrier_ok:
        return "carrier_only"
    if brd_ok:
        return "brd_only"
    return "neither"


def case_payload(
    carrier: dict[str, dict[str, str]],
    brd: dict[str, dict[str, str]],
    names: list[str],
) -> list[dict]:
    cases = []
    for name in names:
        left = carrier.get(name)
        right = brd.get(name)
        left_ms = first_ms(left)
        right_ms = first_ms(right)
        family = (left or right or {}).get("family", "unknown")
        cases.append({
            "name": name,
            "family": family,
            "carrier": {"solved": left_ms is not None, "ms": left_ms},
            "brd": {"solved": right_ms is not None, "ms": right_ms},
            "group": group_for(left, right),
        })
    return cases


def example_cases(cases: list[dict]) -> list[dict]:
    both = [
        case for case in cases
        if case["carrier"]["solved"] and case["brd"]["solved"]
    ]
    carrier_advantage = max(
        both,
        key=lambda case: case["brd"]["ms"] / max(case["carrier"]["ms"], PLOT_FLOOR_MS),
    )
    brd_advantage = max(
        both,
        key=lambda case: case["carrier"]["ms"] / max(case["brd"]["ms"], PLOT_FLOOR_MS),
    )
    carrier_only = next(
        (case for case in cases if case["group"] == "carrier_only"),
        None,
    )
    return [case for case in [carrier_advantage, brd_advantage, carrier_only] if case]


def report_html(payload: dict) -> str:
    data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    return """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Carrier vs carrier_brd：首解时间对比</title>
  <style>
    :root {
      --ink: #172033;
      --muted: #586174;
      --paper: #f7f8fb;
      --card: #ffffff;
      --line: #dce1ea;
      --carrier: #1776b6;
      --brd: #c15c32;
      --green: #2f875d;
      --purple: #7656b3;
      --gray: #7a8493;
      --gold: #a87112;
      --shadow: 0 8px 25px rgba(35, 47, 70, .08);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0; background: var(--paper); color: var(--ink);
      font: 15px/1.62 ui-sans-serif, -apple-system, BlinkMacSystemFont,
        "Segoe UI", "PingFang SC", "Hiragino Sans GB", "Microsoft YaHei", sans-serif;
    }
    .page { max-width: 1280px; margin: 0 auto; padding: 30px 22px 54px; }
    header { padding: 7px 2px 20px; }
    .eyebrow {
      margin: 0 0 7px; color: var(--carrier); font-size: 13px; font-weight: 750;
      letter-spacing: .045em; text-transform: uppercase;
    }
    h1 { margin: 0; font-size: clamp(26px, 4vw, 40px); letter-spacing: -.03em; line-height: 1.14; }
    .lede { max-width: 900px; margin: 12px 0 0; color: var(--muted); font-size: 17px; }
    .notice {
      margin-top: 17px; padding: 12px 15px; border: 1px solid #e1c78c;
      background: #fff9ed; color: #5a4317; border-radius: 9px;
    }
    .section { margin-top: 30px; }
    h2 { margin: 0 0 12px; font-size: 21px; letter-spacing: -.015em; }
    h3 { margin: 0 0 5px; font-size: 16px; }
    .subtle { margin: -5px 0 15px; color: var(--muted); }
    .cards {
      display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 12px;
    }
    .card, .panel {
      background: var(--card); border: 1px solid var(--line); border-radius: 12px;
      box-shadow: var(--shadow);
    }
    .card { padding: 15px; min-height: 116px; }
    .card .label { color: var(--muted); font-size: 13px; }
    .card .number { margin-top: 5px; font-size: 28px; font-weight: 760; letter-spacing: -.035em; }
    .card .hint { color: var(--muted); font-size: 13px; }
    .carrier { color: var(--carrier); }
    .brd { color: var(--brd); }
    .green { color: var(--green); }
    .compare {
      display: grid; grid-template-columns: 1fr 1fr; gap: 16px;
    }
    .panel { padding: 18px; }
    .panel p:last-child { margin-bottom: 0; }
    .metric-table { width: 100%; border-collapse: collapse; font-size: 14px; }
    .metric-table th, .metric-table td {
      padding: 8px 6px; border-bottom: 1px solid #e7eaf0; text-align: right;
      font-variant-numeric: tabular-nums;
    }
    .metric-table th:first-child, .metric-table td:first-child { text-align: left; }
    .metric-table th { color: var(--muted); font-weight: 650; }
    .metric-table tr:last-child td { border-bottom: 0; }
    .chart-shell { padding: 15px 16px 12px; }
    .chart-top { display: flex; gap: 12px; align-items: start; justify-content: space-between; }
    .chart-top p { margin: 3px 0 0; color: var(--muted); font-size: 13px; }
    #scatter { width: 100%; height: auto; display: block; cursor: crosshair; touch-action: none; }
    .legend { display: flex; flex-wrap: wrap; gap: 9px 15px; margin: 9px 2px 1px; color: var(--muted); font-size: 13px; }
    .key { display: inline-flex; gap: 6px; align-items: center; }
    .dot { width: 9px; height: 9px; border-radius: 50%; display: inline-block; }
    .controls {
      display: grid; grid-template-columns: 1.1fr .9fr .9fr; gap: 10px; margin-bottom: 12px;
    }
    label { color: var(--muted); font-size: 13px; }
    input, select {
      width: 100%; margin-top: 4px; padding: 9px 10px; border: 1px solid #ccd3df;
      border-radius: 7px; background: #fff; color: var(--ink); font: inherit;
    }
    .case-layout { display: grid; grid-template-columns: 1.06fr .94fr; gap: 16px; }
    .table-wrap { max-height: 495px; overflow: auto; border: 1px solid #e4e8ef; border-radius: 8px; }
    table { width: 100%; border-collapse: collapse; font-size: 13px; }
    td, th { padding: 8px 9px; border-bottom: 1px solid #edf0f4; text-align: right; font-variant-numeric: tabular-nums; }
    td:first-child, th:first-child { text-align: left; }
    thead th { position: sticky; top: 0; z-index: 1; background: #f2f5f8; color: #4b5669; font-weight: 700; }
    tbody tr { cursor: pointer; }
    tbody tr:hover, tbody tr.selected { background: #edf6fc; }
    .status { font-weight: 700; }
    .status.carrier_faster { color: var(--carrier); }
    .status.brd_faster { color: var(--brd); }
    .status.carrier_only { color: var(--green); }
    .status.brd_only { color: var(--purple); }
    .status.neither { color: var(--gray); }
    .detail { min-height: 300px; }
    .detail .case-name { font-family: ui-monospace, SFMono-Regular, Consolas, monospace; overflow-wrap: anywhere; }
    .detail-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin: 14px 0; }
    .side {
      padding: 11px; border-radius: 8px; background: #f6f8fb; border: 1px solid #e2e7ef;
    }
    .side b { display: block; font-size: 19px; margin-top: 3px; }
    .explain { color: var(--muted); }
    .examples { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 12px; }
    button.example {
      text-align: left; padding: 14px; background: var(--card); color: inherit; border: 1px solid var(--line);
      border-radius: 10px; cursor: pointer; font: inherit; box-shadow: var(--shadow);
    }
    button.example:hover { border-color: #8ebadd; }
    button.example code { display: block; margin: 5px 0; white-space: normal; overflow-wrap: anywhere; color: var(--ink); }
    .method ul { margin: 8px 0 0; padding-left: 20px; }
    .footnote { margin-top: 12px; color: var(--muted); font-size: 13px; }
    code { font-family: ui-monospace, SFMono-Regular, Consolas, monospace; font-size: .91em; }
    @media (max-width: 850px) {
      .cards { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .compare, .case-layout { grid-template-columns: 1fr; }
      .examples { grid-template-columns: 1fr; }
    }
    @media (max-width: 540px) {
      .page { padding: 20px 13px 36px; }
      .cards { grid-template-columns: 1fr; }
      .controls { grid-template-columns: 1fr; }
      .panel { padding: 14px; }
      td, th { padding: 7px 6px; }
    }
  </style>
</head>
<body>
<main class="page">
  <header>
    <p class="eyebrow">First-solution only · 2026-09-09</p>
    <h1><span class="carrier">Carrier（我们的方法）</span> vs <span class="brd">carrier_brd（baseline）</span></h1>
    <p class="lede">这页只比较每个 testcase 的首个可行解时间 <code>first_solution_ms</code>。两边都解出的案例直接比较首解；某一边 10 秒内未解出时，只用于成功率分类，不把 final、deliverable 或总运行时间混进首解比较。</p>
    <div class="notice">当前 518-case 结果是实验快照：两个 method 用同一二进制、同一 manifest、同一 seed 和同一 10 秒上限；但 <code>carrier_brd</code> 已包含尚未提交的 complete-replan 改动。因此它能回答“当前版本谁更快”，不能当作与旧报告完全同版本的因果对照。</div>
  </header>

  <section class="section">
    <h2>当前 full-518：先看结论</h2>
    <p class="subtle" id="run-meta"></p>
    <div class="cards" id="current-cards"></div>
  </section>

  <section class="section compare">
    <div class="panel">
      <h2>为什么和旧网页的结论相反？</h2>
      <p class="explain">旧报告与当前报告并不是同一组 method 版本。为了避免把“代码更新”和“方法优劣”混为一谈，下表固定在旧报告的 509 个 testcase 上比较。</p>
      <table class="metric-table" id="history-table"></table>
      <p class="footnote">几何平均比值 = Carrier 首解时间 / BRD 首解时间；小于 1 表示 Carrier 更快。只统计双方都解出的案例；唯一一个 0 ms / 0 ms 的平凡案例保留在“相同”计数中，但不纳入乘法比值。</p>
    </div>
    <div class="panel method">
      <h2>这次 Carrier 为什么变快？</h2>
      <p id="carrier-change"></p>
      <ul>
        <li>Carrier 在 <code>2d4563c</code> 中加入了增量式 vacancy potential / storage topology 复用，以及增量 pair-cost 与匹配复用。</li>
        <li>同一批旧案例中，vacancy-potential 累计时间从 <b>404,366.8 ms</b> 降到 <b>32,841.0 ms</b>；这是最直接的计时证据。</li>
        <li>同时，baseline 也因 complete-replan 改动改变了成功集合和首解时间，所以当前共同解出子集从旧报告的 327 个变为 457 个。</li>
      </ul>
      <p class="footnote">因此，“当前 Carrier 更快”是可观察到的结果；“完全由哪一项优化贡献多少”仍需要逐项 ablation 才能下定论。</p>
    </div>
  </section>

  <section class="section">
    <h2>518 个 testcase 的首解散点图</h2>
    <p class="subtle">横轴是 baseline 的首解，纵轴是 Carrier 的首解，均为对数坐标。蓝点在对角线下方表示 Carrier 更快；橙点在上方表示 baseline 更快。未解出按 10,000 ms 画在边界上，只表达“超时”，不表达真实首解时间。</p>
    <div class="panel chart-shell">
      <div class="chart-top">
        <div><h3>点击点可在下方定位 testcase</h3><p>筛选条件会同时作用于图和表。</p></div>
        <div id="chart-count" class="subtle"></div>
      </div>
      <svg id="scatter" viewBox="0 0 940 545" role="img" aria-label="Carrier and baseline first-solution time scatter plot"></svg>
      <div class="legend">
        <span class="key"><i class="dot" style="background:#1776b6"></i>双方解出，Carrier 更快</span>
        <span class="key"><i class="dot" style="background:#c15c32"></i>双方解出，baseline 更快</span>
        <span class="key"><i class="dot" style="background:#2f875d"></i>只有 Carrier 解出</span>
        <span class="key"><i class="dot" style="background:#7656b3"></i>只有 baseline 解出</span>
        <span class="key"><i class="dot" style="background:#7a8493"></i>双方未解出</span>
      </div>
    </div>
  </section>

  <section class="section">
    <h2>逐例查看</h2>
    <div class="controls">
      <label>按名称搜索<input id="query" type="search" placeholder="例如 g5_dhigh 或 brap_h80"></label>
      <label>结果类别<select id="group-filter"></select></label>
      <label>案例家族<select id="family-filter"></select></label>
    </div>
    <div class="case-layout">
      <div class="panel">
        <div class="table-wrap">
          <table>
            <thead><tr><th>testcase</th><th>Carrier 首解</th><th>BRD 首解</th><th>结果</th></tr></thead>
            <tbody id="case-table"></tbody>
          </table>
        </div>
      </div>
      <aside class="panel detail" id="detail"></aside>
    </div>
  </section>

  <section class="section">
    <h2>三个可复查的例子</h2>
    <p class="subtle">点击例子会跳到该 testcase 的逐例说明。它们用于帮助读图，不代替全量统计。</p>
    <div class="examples" id="examples"></div>
  </section>

  <section class="section panel method">
    <h2>读这页时要守住的三个边界</h2>
    <ul>
      <li><b>名称不要搞反：</b><code>carrier</code> 是我们的方法；<code>carrier_brd</code> 是 baseline。</li>
      <li><b>只看首解：</b>两边都使用 <code>first_solution_ms</code>；不会拿 baseline 的 final 或运行时去替代它。</li>
      <li><b>旧 vs 新不是严格 ablation：</b>当前 Carrier 有增量 vacancy 优化，当前 BRD 也有 complete-replan 改动。页面同时展示旧 509 和当前 518，目的是解释差异，而不是声称两次实验只变了一个因素。</li>
    </ul>
  </section>
</main>

<script>
const REPORT = __REPORT_DATA__;
const CURRENT = REPORT.current;
const CASES = REPORT.cases;
let selected = REPORT.examples[0] ? REPORT.examples[0].name : CASES[0].name;

const fmt = value => value === null || value === undefined ? "10 秒内未解出" :
  (Number.isInteger(value) ? value.toLocaleString("zh-CN") : value.toLocaleString("zh-CN", {maximumFractionDigits: 1})) + " ms";
const ratio = value => value === null ? "—" : value.toFixed(3) + "×";
const byName = new Map(CASES.map(item => [item.name, item]));

function labelFor(group) {
  return {
    carrier_faster: "Carrier 更快",
    brd_faster: "baseline 更快",
    equal: "首解相同",
    carrier_only: "只有 Carrier 解出",
    brd_only: "只有 baseline 解出",
    neither: "双方未解出",
  }[group];
}

function escapeHtml(value) {
  const node = document.createElement("span");
  node.textContent = value;
  return node.innerHTML;
}

function fillSummary() {
  const stat = CURRENT.summary;
  document.querySelector("#run-meta").textContent =
    `${CURRENT.total} 个案例；每例 10 秒；seed ${CURRENT.seed}；${CURRENT.jobs} 并行任务；` +
    `Carrier 与 BRD 使用相同二进制 ${CURRENT.binary_sha.slice(0, 12)}…`;
  const cards = [
    ["Carrier 解出", `${stat.carrier} / ${stat.total}`, "carrier（我们的方法）", "carrier"],
    ["baseline 解出", `${stat.brd} / ${stat.total}`, "carrier_brd", "brd"],
    ["双方都解出", stat.both, `Carrier 更快 ${stat.carrier_faster}，baseline 更快 ${stat.carrier_slower}，相同 ${stat.equal}`, ""],
    ["共同解出的几何平均", ratio(stat.carrier_over_brd_geomean), "Carrier / baseline；越小越好", stat.carrier_over_brd_geomean < 1 ? "green" : "brd"],
  ];
  document.querySelector("#current-cards").innerHTML = cards.map(([label, number, hint, color]) =>
    `<div class="card"><div class="label">${label}</div><div class="number ${color}">${number}</div><div class="hint">${hint}</div></div>`
  ).join("");

  const old = REPORT.old_509.summary;
  const current509 = REPORT.current_509.summary;
  document.querySelector("#history-table").innerHTML = `
    <thead><tr><th>固定在旧 509 个案例</th><th>旧报告<br>2026-09-07</th><th>当前代码<br>2026-09-09</th></tr></thead>
    <tbody>
      <tr><td>Carrier 解出</td><td>${old.carrier} / ${old.total}</td><td>${current509.carrier} / ${current509.total}</td></tr>
      <tr><td>baseline 解出</td><td>${old.brd} / ${old.total}</td><td>${current509.brd} / ${current509.total}</td></tr>
      <tr><td>双方都解出</td><td>${old.both}</td><td>${current509.both}</td></tr>
      <tr><td>Carrier 更快 / baseline 更快</td><td>${old.carrier_faster} / ${old.carrier_slower}</td><td>${current509.carrier_faster} / ${current509.carrier_slower}</td></tr>
      <tr><td>几何平均比值（Carrier / BRD）</td><td>${ratio(old.carrier_over_brd_geomean)}</td><td>${ratio(current509.carrier_over_brd_geomean)}</td></tr>
    </tbody>`;

  const longitudinal = REPORT.carrier_longitudinal;
  document.querySelector("#carrier-change").innerHTML =
    `在旧 509 个案例中，Carrier 新旧都解出的 ${longitudinal.paired} 个案例里，` +
    `当前首解总和为 <b>${fmt(longitudinal.new_sum_ms)}</b>，旧版本为 <b>${fmt(longitudinal.old_sum_ms)}</b>；` +
    `当前 / 旧版的几何平均比值为 <b>${ratio(longitudinal.new_over_old_geomean)}</b>。` +
    `其中当前更快 ${longitudinal.new_faster} 个、变慢 ${longitudinal.new_slower} 个。`;
}

function fillFilters() {
  const groups = [
    ["all", "全部结果"],
    ["carrier_faster", "双方解出：Carrier 更快"],
    ["brd_faster", "双方解出：baseline 更快"],
    ["equal", "双方解出：首解相同"],
    ["carrier_only", "只有 Carrier 解出"],
    ["brd_only", "只有 baseline 解出"],
    ["neither", "双方未解出"],
  ];
  document.querySelector("#group-filter").innerHTML = groups
    .map(([value, text]) => `<option value="${value}">${text}</option>`).join("");
  const families = [...new Set(CASES.map(item => item.family))].sort();
  document.querySelector("#family-filter").innerHTML =
    `<option value="all">全部案例家族</option>` +
    families.map(family => `<option value="${escapeHtml(family)}">${escapeHtml(family)}</option>`).join("");
}

function visibleCases() {
  const q = document.querySelector("#query").value.trim().toLowerCase();
  const group = document.querySelector("#group-filter").value;
  const family = document.querySelector("#family-filter").value;
  return CASES.filter(item =>
    (!q || item.name.toLowerCase().includes(q) || item.family.toLowerCase().includes(q)) &&
    (group === "all" || item.group === group) &&
    (family === "all" || item.family === family)
  );
}

function renderTable(items) {
  const body = document.querySelector("#case-table");
  body.innerHTML = items.map(item => `
    <tr data-name="${escapeHtml(item.name)}" class="${item.name === selected ? "selected" : ""}">
      <td><code>${escapeHtml(item.name)}</code></td>
      <td>${fmt(item.carrier.ms)}</td>
      <td>${fmt(item.brd.ms)}</td>
      <td><span class="status ${item.group}">${labelFor(item.group)}</span></td>
    </tr>`).join("");
  for (const row of body.querySelectorAll("tr")) {
    row.addEventListener("click", () => selectCase(row.dataset.name));
  }
}

function renderDetail() {
  const item = byName.get(selected);
  if (!item) {
    document.querySelector("#detail").innerHTML = "<p>当前筛选下没有选中的 testcase。</p>";
    return;
  }
  const both = item.carrier.solved && item.brd.solved;
  const relative = both
    ? `Carrier / baseline = <b>${ratio(item.carrier.ms / Math.max(item.brd.ms, .5))}</b>。`
    : item.group === "carrier_only"
      ? "Carrier 已得到首解，而 baseline 在 10 秒内没有首解。"
      : item.group === "brd_only"
        ? "baseline 已得到首解，而 Carrier 在 10 秒内没有首解。"
        : "两边都在 10 秒内没有得到首解。";
  document.querySelector("#detail").innerHTML = `
    <h3 class="case-name">${escapeHtml(item.name)}</h3>
    <p class="subtle">${escapeHtml(item.family)} · <span class="status ${item.group}">${labelFor(item.group)}</span></p>
    <div class="detail-grid">
      <div class="side"><span class="carrier">Carrier（我们的方法）</span><b>${fmt(item.carrier.ms)}</b></div>
      <div class="side"><span class="brd">carrier_brd（baseline）</span><b>${fmt(item.brd.ms)}</b></div>
    </div>
    <p>${relative}</p>
    <p class="explain">这里严格是首解时间，不引用最终解、执行完成时间或 solver 总运行时间。散点图中的边界点只表示 10 秒超时。</p>`;
}

function prettyExample(item, index) {
  const titles = ["Carrier 优势最大的共同解出案例", "baseline 优势最大的共同解出案例", "Carrier 单独解出的案例"];
  return `<button class="example" data-name="${escapeHtml(item.name)}">
    <strong>${titles[index]}</strong>
    <code>${escapeHtml(item.name)}</code>
    <span class="explain">Carrier ${fmt(item.carrier.ms)}；baseline ${fmt(item.brd.ms)}</span>
  </button>`;
}

function renderExamples() {
  document.querySelector("#examples").innerHTML = REPORT.examples.map(prettyExample).join("");
  for (const button of document.querySelectorAll(".example")) {
    button.addEventListener("click", () => selectCase(button.dataset.name));
  }
}

function selectCase(name) {
  selected = name;
  const item = byName.get(name);
  if (item) {
    document.querySelector("#query").value = "";
    document.querySelector("#group-filter").value = "all";
    document.querySelector("#family-filter").value = "all";
  }
  renderAll();
  document.querySelector("#detail").scrollIntoView({behavior: "smooth", block: "nearest"});
}

function renderScatter(items) {
  const svg = document.querySelector("#scatter");
  const left = 97, top = 27, right = 42, bottom = 74, width = 940 - left - right, height = 545 - top - bottom;
  const logMin = Math.log10(.5), logMax = 4;
  const projectX = value => left + (Math.log10(Math.max(value, .5)) - logMin) / (logMax - logMin) * width;
  const projectY = value => top + height - (Math.log10(Math.max(value, .5)) - logMin) / (logMax - logMin) * height;
  const point = item => ({
    x: projectX(item.brd.ms ?? 10000),
    y: projectY(item.carrier.ms ?? 10000),
  });
  const ticks = [.5, 1, 10, 100, 1000, 10000];
  const tickText = value => value === 10000 ? "10 s" : value < 1 ? "0.5" : `${value}`;
  const colors = {
    carrier_faster: "#1776b6", brd_faster: "#c15c32", equal: "#a87112",
    carrier_only: "#2f875d", brd_only: "#7656b3", neither: "#7a8493",
  };
  const grid = ticks.map(value => {
    const x = projectX(value), y = projectY(value);
    return `<line x1="${x}" y1="${top}" x2="${x}" y2="${top + height}" class="grid"/>` +
      `<line x1="${left}" y1="${y}" x2="${left + width}" y2="${y}" class="grid"/>` +
      `<text x="${x}" y="${top + height + 23}" class="tick" text-anchor="middle">${tickText(value)}</text>` +
      `<text x="${left - 10}" y="${y + 4}" class="tick" text-anchor="end">${tickText(value)}</text>`;
  }).join("");
  const points = items.map(item => {
    const p = point(item);
    const active = item.name === selected;
    return `<circle data-name="${escapeHtml(item.name)}" cx="${p.x}" cy="${p.y}" r="${active ? 6 : 4.2}"
      fill="${colors[item.group]}" opacity="${active ? 1 : .82}" stroke="${active ? "#101827" : "#fff"}"
      stroke-width="${active ? 2.1 : 1.1}"><title>${escapeHtml(item.name)} — ${labelFor(item.group)}</title></circle>`;
  }).join("");
  svg.innerHTML = `<style>
      .grid{stroke:#e1e6ee;stroke-width:1}.axis{stroke:#6c778a;stroke-width:1.25}
      .diagonal{stroke:#9ba6b7;stroke-dasharray:6 5;stroke-width:1.3}.tick{fill:#647085;font:12px ui-sans-serif,sans-serif}
      .axislabel{fill:#354055;font:14px ui-sans-serif,sans-serif;font-weight:650}
    </style>
    <rect x="${left}" y="${top}" width="${width}" height="${height}" fill="#fbfcfe" stroke="#d9e0ea"/>
    ${grid}
    <line x1="${left}" y1="${top + height}" x2="${left + width}" y2="${top}" class="diagonal"/>
    <line x1="${left}" y1="${top + height}" x2="${left + width}" y2="${top + height}" class="axis"/>
    <line x1="${left}" y1="${top}" x2="${left}" y2="${top + height}" class="axis"/>
    <text x="${left + width / 2}" y="530" class="axislabel" text-anchor="middle">carrier_brd（baseline）首解时间 / ms</text>
    <text x="19" y="${top + height / 2}" class="axislabel" text-anchor="middle" transform="rotate(-90 19 ${top + height / 2})">Carrier（我们的方法）首解时间 / ms</text>
    ${points}`;
  for (const node of svg.querySelectorAll("circle[data-name]")) {
    node.addEventListener("click", () => selectCase(node.dataset.name));
  }
  document.querySelector("#chart-count").textContent = `显示 ${items.length} / ${CASES.length} 个案例`;
}

function renderAll() {
  const items = visibleCases();
  renderScatter(items);
  renderTable(items);
  renderDetail();
}

function init() {
  fillSummary();
  fillFilters();
  renderExamples();
  for (const id of ["query", "group-filter", "family-filter"]) {
    document.querySelector(`#${id}`).addEventListener("input", renderAll);
    document.querySelector(`#${id}`).addEventListener("change", renderAll);
  }
  renderAll();
}
init();
</script>
</body>
</html>
""".replace("__REPORT_DATA__", data)


def main() -> None:
    repo = Path(__file__).resolve().parents[1]
    sibling = repo.parent / "dd-lacam"
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--carrier-run",
        type=Path,
        default=repo / "benchmark/results_full_carrier_baseline_complete_replan_20260909",
        help="current carrier result directory",
    )
    parser.add_argument(
        "--brd-run",
        type=Path,
        default=repo / "benchmark/results_full_carrier_brd_complete_replan_20260909",
        help="current carrier_brd result directory",
    )
    parser.add_argument(
        "--old-carrier-run",
        type=Path,
        default=sibling / "benchmark/results_full_vacancy_shared_carrier_20260907",
        help="historical carrier result directory",
    )
    parser.add_argument(
        "--old-brd-run",
        type=Path,
        default=sibling / "benchmark/results_full_vacancy_shared_carrier_brd_20260907",
        help="historical carrier_brd result directory",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=repo / "benchmark/viz_web/carrier_vs_brd_first_solution_full518_20260909",
    )
    args = parser.parse_args()

    carrier = read_rows(args.carrier_run / "rows.csv")
    brd = read_rows(args.brd_run / "rows.csv")
    old_carrier = read_rows(args.old_carrier_run / "rows.csv")
    old_brd = read_rows(args.old_brd_run / "rows.csv")
    carrier_meta = read_json(args.carrier_run / "timing.json")
    brd_meta = read_json(args.brd_run / "timing.json")

    current_names = sorted(set(carrier) | set(brd))
    old_names = sorted(set(old_carrier) | set(old_brd))
    shared_names = sorted(set(current_names) & set(old_names))
    if len(current_names) != 518:
        raise ValueError(f"expected 518 current cases, got {len(current_names)}")
    if len(old_names) != 509:
        raise ValueError(f"expected 509 historical cases, got {len(old_names)}")
    if len(shared_names) != 509:
        raise ValueError(
            f"historical cases must be contained in current run; intersection is {len(shared_names)}"
        )
    if carrier_meta["provenance"]["binary_sha256"] != brd_meta["provenance"]["binary_sha256"]:
        raise ValueError("current Carrier and BRD runs use different solver binaries")

    current_cases = case_payload(carrier, brd, current_names)
    payload = {
        "current": {
            "total": len(current_names),
            "seed": carrier_meta["solver_seed"],
            "jobs": carrier_meta["jobs"],
            "binary_sha": carrier_meta["provenance"]["binary_sha256"],
            "git_commit": carrier_meta["provenance"]["git_commit"],
            "summary": comparison_summary(carrier, brd, current_names),
        },
        "current_509": {
            "summary": comparison_summary(carrier, brd, shared_names),
        },
        "old_509": {
            "summary": comparison_summary(old_carrier, old_brd, old_names),
        },
        "carrier_longitudinal": longitudinal_summary(old_carrier, carrier, shared_names),
        "cases": current_cases,
        "examples": example_cases(current_cases),
    }
    args.out_dir.mkdir(parents=True, exist_ok=True)
    output = args.out_dir / "index.html"
    output.write_text(report_html(payload), encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
