#!/usr/bin/env python3
"""Generate an all-case dashboard comparing two full benchmark runs."""

import argparse
import csv
import html
import json
import math
import os
import statistics
from collections import defaultdict
from pathlib import Path


AXES = (
    "map_family",
    "density_level",
    "agent_level",
    "task_profile",
    "goal_mode",
)


def _success(row):
    return str(row.get("success", "")).lower() in {"1", "true", "yes"}


def _number(row, key):
    value = row.get(key, "")
    return float(value) if value not in ("", None) else None


def _load_rows(path):
    with Path(path).open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    names = [row["instance"] for row in rows]
    if len(names) != len(set(names)):
        raise ValueError("duplicate instance names in {}".format(path))
    return rows, {row["instance"]: row for row in rows}


def _metric(records, baseline_key, current_key):
    comparable = [
        record
        for record in records
        if record["baseline_success"] and record["current_success"]
    ]
    better = equal = worse = 0
    ratios = []
    baseline_sum = current_sum = 0.0
    for record in comparable:
        baseline = record[baseline_key]
        current = record[current_key]
        baseline_sum += baseline
        current_sum += current
        if current < baseline:
            better += 1
        elif current == baseline:
            equal += 1
        else:
            worse += 1
        if baseline > 0 and current > 0:
            ratios.append(current / baseline)
    geometric_ratio = (
        math.exp(
            sum(math.log(value) for value in ratios) / len(ratios)
        )
        if ratios
        else None
    )
    return {
        "better_equal_worse": [better, equal, worse],
        "baseline_sum": baseline_sum,
        "current_sum": current_sum,
        "sum_delta": current_sum - baseline_sum,
        "geometric_ratio": geometric_ratio,
    }


def _summary(records):
    baseline_solved = sum(record["baseline_success"] for record in records)
    current_solved = sum(record["current_success"] for record in records)
    common = [
        record
        for record in records
        if record["baseline_success"] and record["current_success"]
    ]
    lex_counts = [
        sum(record["verdict"] == verdict for record in common)
        for verdict in ("better", "equal", "worse")
    ]
    baseline_runtimes = [
        record["baseline_runtime"]
        for record in records
        if record["baseline_runtime"] is not None
    ]
    current_runtimes = [
        record["current_runtime"]
        for record in records
        if record["current_runtime"] is not None
    ]
    return {
        "total": len(records),
        "baseline_solved": baseline_solved,
        "current_solved": current_solved,
        "common_solved": len(common),
        "gained": sum(record["verdict"] == "gained" for record in records),
        "lost": sum(record["verdict"] == "lost" for record in records),
        "both_failed": sum(
            record["verdict"] == "both_failed" for record in records
        ),
        "success_sets_equal": all(
            record["baseline_success"] == record["current_success"]
            for record in records
        ),
        "makespan": _metric(
            records, "baseline_makespan", "current_makespan"
        ),
        "work": _metric(records, "baseline_work", "current_work"),
        "lexicographic": {
            "better_equal_worse": lex_counts,
        },
        "plan_hash_changes": sum(
            record["baseline_plan_sha256"]
            != record["current_plan_sha256"]
            for record in common
        ),
        "baseline_runtime_sum": sum(baseline_runtimes),
        "current_runtime_sum": sum(current_runtimes),
        "baseline_runtime_mean": (
            statistics.mean(baseline_runtimes)
            if baseline_runtimes
            else None
        ),
        "current_runtime_mean": (
            statistics.mean(current_runtimes)
            if current_runtimes
            else None
        ),
    }


def _group_summaries(records, key):
    groups = defaultdict(list)
    for record in records:
        value = record.get(key)
        if value not in (None, ""):
            groups[str(value)].append(record)
    return [
        {"key": key_value, **_summary(items)}
        for key_value, items in sorted(groups.items())
    ]


def _timing_payload(timing):
    wall = float(timing["wall_time_sec"])
    solver_sum = float(
        timing["methods"]["carrier"]["solver_time_sum_sec"]
    )
    return {
        "wall_time_sec": wall,
        "solver_time_sum_sec": solver_sum,
        "binary_sha256": timing["provenance"]["binary_sha256"],
        "suite_sha256": timing["suite"]["definition_sha256"],
    }


def build_comparison_data(
    baseline_rows_path,
    baseline_timing_path,
    current_rows_path,
    current_timing_path,
    manifest_path,
    baseline_case_prefix,
    current_case_prefix,
):
    baseline_rows, baseline = _load_rows(baseline_rows_path)
    _, current = _load_rows(current_rows_path)
    if set(baseline) != set(current):
        missing_current = sorted(set(baseline) - set(current))
        missing_baseline = sorted(set(current) - set(baseline))
        raise ValueError(
            "case sets differ: missing current {}; missing baseline {}".format(
                missing_current[:5], missing_baseline[:5]
            )
        )

    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    metadata = {record["id"]: record for record in manifest}
    records = []
    for baseline_row in baseline_rows:
        name = baseline_row["instance"]
        current_row = current[name]
        baseline_success = _success(baseline_row)
        current_success = _success(current_row)
        baseline_makespan = _number(
            baseline_row, "executed_makespan"
        )
        current_makespan = _number(current_row, "executed_makespan")
        baseline_work = _number(baseline_row, "weighted_soc")
        current_work = _number(current_row, "weighted_soc")
        if baseline_success and current_success:
            baseline_cost = (baseline_makespan, baseline_work)
            current_cost = (current_makespan, current_work)
            if current_cost < baseline_cost:
                verdict = "better"
            elif current_cost == baseline_cost:
                verdict = "equal"
            else:
                verdict = "worse"
        elif current_success:
            verdict = "gained"
        elif baseline_success:
            verdict = "lost"
        else:
            verdict = "both_failed"
        meta = metadata.get(name, {})
        records.append(
            {
                "instance": name,
                "scope": "factorial" if name in metadata else "quick",
                "family": current_row.get(
                    "family", baseline_row.get("family", "")
                ),
                **{axis: meta.get(axis) for axis in AXES},
                "baseline_success": baseline_success,
                "current_success": current_success,
                "baseline_status": baseline_row.get("status", ""),
                "current_status": current_row.get("status", ""),
                "baseline_makespan": baseline_makespan,
                "current_makespan": current_makespan,
                "makespan_delta": (
                    current_makespan - baseline_makespan
                    if baseline_success and current_success
                    else None
                ),
                "baseline_work": baseline_work,
                "current_work": current_work,
                "work_delta": (
                    current_work - baseline_work
                    if baseline_success and current_success
                    else None
                ),
                "baseline_runtime": _number(
                    baseline_row, "runtime_sec"
                ),
                "current_runtime": _number(current_row, "runtime_sec"),
                "baseline_plan_sha256": baseline_row.get(
                    "plan_sha256", ""
                ),
                "current_plan_sha256": current_row.get(
                    "plan_sha256", ""
                ),
                "verdict": verdict,
                "baseline_animation": (
                    "{}/{}.html".format(
                        baseline_case_prefix.rstrip("/"), name
                    )
                    if baseline_success
                    else None
                ),
                "current_animation": (
                    "{}/{}.html".format(
                        current_case_prefix.rstrip("/"), name
                    )
                    if current_success
                    else None
                ),
            }
        )

    baseline_timing = _timing_payload(
        json.loads(Path(baseline_timing_path).read_text(encoding="utf-8"))
    )
    current_timing = _timing_payload(
        json.loads(Path(current_timing_path).read_text(encoding="utf-8"))
    )
    if baseline_timing["suite_sha256"] != current_timing["suite_sha256"]:
        raise ValueError("suite definition hashes differ")
    factorial = [record for record in records if record["scope"] == "factorial"]
    quick = [record for record in records if record["scope"] == "quick"]
    timing = {
        "baseline": baseline_timing,
        "current": current_timing,
        "wall_ratio": (
            current_timing["wall_time_sec"]
            / baseline_timing["wall_time_sec"]
        ),
        "solver_sum_ratio": (
            current_timing["solver_time_sum_sec"]
            / baseline_timing["solver_time_sum_sec"]
        ),
    }
    return {
        "overview": _summary(records),
        "quick": _summary(quick),
        "factorial": _summary(factorial),
        "timing": timing,
        "groups": {
            "scope": _group_summaries(records, "scope"),
            "family": _group_summaries(records, "family"),
            **{
                axis: _group_summaries(factorial, axis)
                for axis in AXES
            },
        },
        "cases": records,
    }


def _fmt(value, digits=3):
    if value is None:
        return "—"
    return ("{0:." + str(digits) + "f}").format(value)


def _integer(value):
    return str(int(round(value)))


def _bwe(summary):
    return " / ".join(
        str(value) for value in summary["better_equal_worse"]
    )


def _group_table(rows):
    output = []
    for row in rows:
        output.append(
            "<tr><td>{}</td><td>{}</td><td>{}/{} → {}/{}</td>"
            "<td>{}</td><td>{}</td><td>{} → {}</td>"
            "<td>{}</td><td>{}</td><td>{} → {}</td></tr>".format(
                html.escape(row["key"]),
                row["total"],
                row["baseline_solved"],
                row["total"],
                row["current_solved"],
                row["total"],
                _bwe(row["makespan"]),
                _fmt(row["makespan"]["geometric_ratio"]),
                _integer(row["makespan"]["baseline_sum"]),
                _integer(row["makespan"]["current_sum"]),
                _bwe(row["work"]),
                _fmt(row["work"]["geometric_ratio"]),
                _integer(row["work"]["baseline_sum"]),
                _integer(row["work"]["current_sum"]),
            )
        )
    return "\n".join(output)


def _relative_url(path, output):
    return Path(
        os.path.relpath(Path(path).resolve(), Path(output).resolve())
    ).as_posix()


def _js_template_text(value):
    return (
        html.escape(str(value))
        .replace("\\", "\\\\")
        .replace("`", "\\`")
        .replace("${", "\\${")
    )


def _navigation_links(
    baseline_label,
    current_label,
    baseline_rows_url,
    current_rows_url,
    report_url,
    baseline_dashboard_url,
    current_dashboard_url,
):
    links = [("../index.html", "可视化首页")]
    if report_url:
        links.append((report_url, "本次最终报告"))
    if baseline_dashboard_url:
        links.append(
            (baseline_dashboard_url, "{} dashboard".format(baseline_label))
        )
    if current_dashboard_url:
        links.append(
            (current_dashboard_url, "{} dashboard".format(current_label))
        )
    links.extend(
        (
            (baseline_rows_url, "{} rows.csv".format(baseline_label)),
            (current_rows_url, "{} rows.csv".format(current_label)),
        )
    )
    return "\n  ".join(
        '<a href="{}">{}</a>'.format(
            html.escape(str(url), quote=True),
            html.escape(str(label)),
        )
        for url, label in links
    )


def _generate_html(
    data,
    baseline_label,
    current_label,
    baseline_rows_url,
    current_rows_url,
    report_url=None,
    baseline_dashboard_url=None,
    current_dashboard_url=None,
):
    overview = data["overview"]
    timing = data["timing"]
    cases_json = json.dumps(
        data["cases"], ensure_ascii=False, separators=(",", ":")
    ).replace("<", "\\u003c")
    families = sorted({record["family"] for record in data["cases"]})
    family_options = "".join(
        '<option value="{}">{}</option>'.format(
            html.escape(family), html.escape(family)
        )
        for family in families
    )
    return """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Full 509 对比：{baseline_label} vs {current_label}</title>
<style>
:root {{ --bg:#07101b; --panel:#0d1c2d; --line:#28445f; --text:#edf6ff;
  --muted:#9fb3c7; --good:#2dd4bf; --bad:#fb7185; --same:#94a3b8;
  --blue:#60a5fa; --amber:#fbbf24; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; color:var(--text); background:
  radial-gradient(circle at 10% 0,#173b61 0,transparent 35rem),
  radial-gradient(circle at 100% 10%,#123d3e 0,transparent 32rem),var(--bg);
  font-family:Inter,ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif;
  line-height:1.55; }}
main {{ max-width:1500px; margin:auto; padding:28px 20px 60px; }}
a {{ color:#93c5fd; }} h1 {{ margin:7px 0 9px; font-size:clamp(30px,4vw,52px); }}
h2 {{ margin:32px 0 12px; }} p {{ color:var(--muted); }}
.eyebrow {{ color:var(--good); font-weight:800; letter-spacing:.12em;
  text-transform:uppercase; font-size:12px; }}
.cards {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(190px,1fr));
  gap:12px; margin:22px 0; }}
.card,.panel {{ border:1px solid var(--line); border-radius:15px;
  background:linear-gradient(145deg,#11253aee,#091724ee);
  box-shadow:0 16px 42px #0004; }}
.card {{ padding:16px; }} .card small {{ color:var(--muted); }}
.card b {{ display:block; font-size:27px; margin:4px 0; }}
.panel {{ padding:18px; margin:14px 0; }} .good {{ color:var(--good); }}
.bad {{ color:var(--bad); }} .same {{ color:var(--same); }}
.two {{ display:grid; grid-template-columns:1fr 1fr; gap:14px; }}
.nav,.filters {{ display:flex; gap:9px; flex-wrap:wrap; align-items:center; }}
.nav a {{ border:1px solid var(--line); padding:8px 11px; border-radius:9px;
  text-decoration:none; background:#091724; }}
select,input {{ background:#081522; color:var(--text); border:1px solid var(--line);
  border-radius:8px; padding:8px 10px; }}
input {{ min-width:260px; }}
.table-wrap {{ overflow:auto; }}
table {{ width:100%; border-collapse:collapse; font-size:12px; }}
th,td {{ border-bottom:1px solid #1d3851; padding:8px 7px; text-align:right;
  white-space:nowrap; }}
th:first-child,td:first-child {{ text-align:left; }}
th {{ position:sticky; top:0; background:#0d1c2d; z-index:1; color:#c9d9e8; }}
canvas {{ display:block; width:100%; height:330px; background:#081522;
  border:1px solid var(--line); border-radius:10px; }}
.legend {{ font-size:12px; color:var(--muted); }}
.pill {{ border-radius:999px; padding:2px 7px; font-weight:700; }}
.pill.better,.pill.gained {{ color:var(--good); background:#0c3b39; }}
.pill.worse,.pill.lost {{ color:var(--bad); background:#481d2a; }}
.pill.equal,.pill.both_failed {{ color:#cbd5e1; background:#263446; }}
.note {{ border-left:4px solid var(--amber); padding:10px 14px;
  background:#241d0d; color:#f8deb0; }}
.foot {{ margin-top:28px; font-size:12px; color:var(--muted); }}
@media(max-width:900px) {{ .two {{ grid-template-columns:1fr; }} }}
</style>
</head>
<body>
<main>
<div class="eyebrow">同一套 full benchmark · 全部 {total} 行</div>
<h1>{baseline_label} vs {current_label}</h1>
<p>逐例配对相同的 {total} 个 case。先比较是否解出；两版都成功时，再按
严格词典序 <b>(T, W)</b> 判断：T（完成时间）优先，T 相同才比较 W。
表中 better/equal/worse 均从 {current_label} 的角度计算，负 Δ 表示更小。</p>
<p class="note">这是两个独立 full benchmark 产物的回顾性配对比较。两次运行
suite SHA 相同，但算法版本和实际运行成本不同；每一侧的完成证据以各自
dashboard、rows.csv、timing.json 和 provenance 为准。</p>
<div class="nav">
  {navigation_links}
</div>

<section class="cards">
  <div class="card"><small>solved</small><b>{base_solved}/{total} → {current_solved}/{total}</b>
    <span class="same">gained {gained} · lost {lost}</span></div>
  <div class="card"><small>严格 (T,W) B / E / W</small><b class="good">{lex_bwe}</b>
    <span class="same">{common} 个共同成功例</span></div>
  <div class="card"><small>Makespan T B / E / W</small><b>{t_bwe}</b>
    <span class="good">几何比 {t_ratio}</span></div>
  <div class="card"><small>Work W B / E / W</small><b>{w_bwe}</b>
    <span class="bad">几何比 {w_ratio}</span></div>
  <div class="card"><small>wall time</small><b>{base_wall}s → {current_wall}s</b>
    <span class="bad">{wall_ratio}×</span></div>
  <div class="card"><small>plan hash changed</small><b>{plan_changes}/{common}</b>
    <span class="same">共同成功例</span></div>
</section>

<div class="two">
  <div class="panel">
    <h2>Makespan：{baseline_label} vs {current_label}</h2>
    <canvas id="scatterT" width="700" height="330"></canvas>
    <div class="legend">横轴为 {baseline_label}，纵轴为 {current_label}；
      对数坐标，对角线下方（绿色）表示 {current_label} 更快。</div>
  </div>
  <div class="panel">
    <h2>Work：{baseline_label} vs {current_label}</h2>
    <canvas id="scatterW" width="700" height="330"></canvas>
    <div class="legend">横轴为 {baseline_label}，纵轴为 {current_label}；
      对数坐标，对角线下方表示 {current_label} 的 work 更小。</div>
  </div>
</div>

<h2>范围汇总</h2>
<div class="panel table-wrap"><table>
<thead><tr><th>范围</th><th>cases</th><th>solved</th>
<th>T B/E/W</th><th>T 几何比</th><th>T sum</th>
<th>W B/E/W</th><th>W 几何比</th><th>W sum</th></tr></thead>
<tbody>{scope_rows}</tbody></table></div>

<h2>Family 汇总</h2>
<div class="panel table-wrap"><table>
<thead><tr><th>family</th><th>cases</th><th>solved</th>
<th>T B/E/W</th><th>T 几何比</th><th>T sum</th>
<th>W B/E/W</th><th>W 几何比</th><th>W sum</th></tr></thead>
<tbody>{family_rows}</tbody></table></div>

<h2>Factorial 五个设计轴</h2>
<div class="panel">
  <select id="axisSelect">
    <option value="map_family">地图规模</option>
    <option value="density_level">货架密度</option>
    <option value="agent_level">机器人数量</option>
    <option value="task_profile">任务分布</option>
    <option value="goal_mode">Goal 模式</option>
  </select>
  <div class="table-wrap"><table>
    <thead><tr><th>组</th><th>cases</th><th>solved</th>
    <th>T B/E/W</th><th>T 几何比</th><th>T sum</th>
    <th>W B/E/W</th><th>W 几何比</th><th>W sum</th></tr></thead>
    <tbody id="axisBody"></tbody>
  </table></div>
</div>

<h2>全部 {total} 行逐例对比</h2>
<div class="panel">
  <div class="filters">
    <input id="search" placeholder="搜索 instance">
    <select id="scopeFilter"><option value="">全部范围</option>
      <option value="quick">quick 77</option>
      <option value="factorial">factorial 432</option></select>
    <select id="verdictFilter"><option value="">全部结论</option>
      <option value="better">better</option><option value="equal">equal</option>
      <option value="worse">worse</option><option value="gained">gained</option>
      <option value="lost">lost</option><option value="both_failed">both failed</option></select>
    <select id="familyFilter"><option value="">全部 family</option>{family_options}</select>
    <span id="visibleCount" class="same"></span>
  </div>
  <div class="table-wrap"><table id="caseTable">
    <thead><tr><th>instance</th><th>范围</th><th>family</th><th>结论</th>
      <th>状态 {baseline_label} → {current_label}</th>
      <th>T {baseline_label} → {current_label}</th><th>ΔT</th>
      <th>W {baseline_label} → {current_label}</th><th>ΔW</th>
      <th>runtime {baseline_label} → {current_label}</th>
      <th>动画</th></tr></thead>
    <tbody id="caseBody"></tbody>
  </table></div>
</div>

<div class="foot">Binary: {base_binary} → {current_binary}<br>
Suite: {suite}<br>
solver-time sum: {base_solver}s → {current_solver}s ({solver_ratio}×)</div>
</main>
<script>
const CASES={cases_json};
const GROUPS={groups_json};
const esc=s=>String(s??"").replace(/[&<>"']/g,c=>({{"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}}[c]));
const num=(v,d=0)=>v==null?"—":Number(v).toFixed(d);
const pair=(a,b,d=0)=>`${{num(a,d)}} → ${{num(b,d)}}`;
const delta=v=>v==null?"—":`${{v>0?"+":""}}${{num(v,0)}}`;
const bwe=m=>m.better_equal_worse.join(" / ");
function groupRows(rows){{
  return rows.map(r=>`<tr><td>${{esc(r.key)}}</td><td>${{r.total}}</td>
    <td>${{r.baseline_solved}}/${{r.total}} → ${{r.current_solved}}/${{r.total}}</td>
    <td>${{bwe(r.makespan)}}</td><td>${{num(r.makespan.geometric_ratio,3)}}</td>
    <td>${{num(r.makespan.baseline_sum)}} → ${{num(r.makespan.current_sum)}}</td>
    <td>${{bwe(r.work)}}</td><td>${{num(r.work.geometric_ratio,3)}}</td>
    <td>${{num(r.work.baseline_sum)}} → ${{num(r.work.current_sum)}}</td></tr>`).join("");
}}
function renderAxis(){{
  document.getElementById("axisBody").innerHTML=
    groupRows(GROUPS[document.getElementById("axisSelect").value]);
}}
function renderCases(){{
  const q=document.getElementById("search").value.toLowerCase();
  const scope=document.getElementById("scopeFilter").value;
  const verdict=document.getElementById("verdictFilter").value;
  const family=document.getElementById("familyFilter").value;
  const rows=CASES.filter(r=>(!q||r.instance.toLowerCase().includes(q))&&
    (!scope||r.scope===scope)&&(!verdict||r.verdict===verdict)&&
    (!family||r.family===family));
  document.getElementById("visibleCount").textContent=`显示 ${{rows.length}} / ${{CASES.length}}`;
  document.getElementById("caseBody").innerHTML=rows.map(r=>{{
    const links=[r.baseline_animation?`<a href="${{esc(r.baseline_animation)}}">{baseline_link_label}</a>`:"",
      r.current_animation?`<a href="${{esc(r.current_animation)}}">{current_link_label}</a>`:""].filter(Boolean).join(" · ");
    return `<tr><td>${{esc(r.instance)}}</td><td>${{r.scope}}</td><td>${{esc(r.family)}}</td>
      <td><span class="pill ${{r.verdict}}">${{r.verdict}}</span></td>
      <td>${{esc(r.baseline_status)}} → ${{esc(r.current_status)}}</td>
      <td>${{pair(r.baseline_makespan,r.current_makespan)}}</td><td>${{delta(r.makespan_delta)}}</td>
      <td>${{pair(r.baseline_work,r.current_work)}}</td><td>${{delta(r.work_delta)}}</td>
      <td>${{pair(r.baseline_runtime,r.current_runtime,3)}}</td><td>${{links||"—"}}</td></tr>`;
  }}).join("");
}}
function scatter(id,baseKey,currentKey){{
  const canvas=document.getElementById(id),ctx=canvas.getContext("2d");
  const rows=CASES.filter(r=>r.baseline_success&&r.current_success);
  const values=rows.flatMap(r=>[r[baseKey],r[currentKey]]).filter(v=>v>0);
  const lo=Math.log10(Math.min(...values)),hi=Math.log10(Math.max(...values));
  const pad=36,w=canvas.width-pad*2,h=canvas.height-pad*2;
  const scale=v=>(Math.log10(v)-lo)/(hi-lo||1);
  ctx.clearRect(0,0,canvas.width,canvas.height); ctx.strokeStyle="#46617b";
  ctx.beginPath();ctx.moveTo(pad,pad+h);ctx.lineTo(pad+w,pad);ctx.stroke();
  ctx.fillStyle="#9fb3c7";ctx.font="11px sans-serif";
  ctx.fillText({baseline_label_js},canvas.width/2,canvas.height-7);
  ctx.save();ctx.translate(11,canvas.height/2);ctx.rotate(-Math.PI/2);
  ctx.fillText({current_label_js},0,0);ctx.restore();
  rows.forEach(r=>{{
    const x=pad+scale(r[baseKey])*w,y=pad+h-scale(r[currentKey])*h;
    ctx.fillStyle=r[currentKey]<r[baseKey]?"#2dd4bf":r[currentKey]>r[baseKey]?"#fb7185":"#94a3b8";
    ctx.globalAlpha=.66;ctx.beginPath();ctx.arc(x,y,2.5,0,Math.PI*2);ctx.fill();
  }});ctx.globalAlpha=1;
}}
["search","scopeFilter","verdictFilter","familyFilter"].forEach(id=>
  document.getElementById(id).addEventListener("input",renderCases));
document.getElementById("axisSelect").addEventListener("change",renderAxis);
renderAxis();renderCases();scatter("scatterT","baseline_makespan","current_makespan");
scatter("scatterW","baseline_work","current_work");
</script>
</body></html>
""".format(
        baseline_label=html.escape(baseline_label),
        current_label=html.escape(current_label),
        baseline_link_label=_js_template_text(baseline_label),
        current_link_label=_js_template_text(current_label),
        baseline_label_js=json.dumps(baseline_label, ensure_ascii=False),
        current_label_js=json.dumps(current_label, ensure_ascii=False),
        navigation_links=_navigation_links(
            baseline_label=baseline_label,
            current_label=current_label,
            baseline_rows_url=baseline_rows_url,
            current_rows_url=current_rows_url,
            report_url=report_url,
            baseline_dashboard_url=baseline_dashboard_url,
            current_dashboard_url=current_dashboard_url,
        ),
        total=overview["total"],
        base_solved=overview["baseline_solved"],
        current_solved=overview["current_solved"],
        gained=overview["gained"],
        lost=overview["lost"],
        common=overview["common_solved"],
        lex_bwe=_bwe(overview["lexicographic"]),
        t_bwe=_bwe(overview["makespan"]),
        t_ratio=_fmt(overview["makespan"]["geometric_ratio"], 6),
        w_bwe=_bwe(overview["work"]),
        w_ratio=_fmt(overview["work"]["geometric_ratio"], 6),
        base_wall=_fmt(timing["baseline"]["wall_time_sec"], 1),
        current_wall=_fmt(timing["current"]["wall_time_sec"], 1),
        wall_ratio=_fmt(timing["wall_ratio"], 2),
        plan_changes=overview["plan_hash_changes"],
        scope_rows=_group_table(data["groups"]["scope"]),
        family_rows=_group_table(data["groups"]["family"]),
        family_options=family_options,
        cases_json=cases_json,
        groups_json=json.dumps(
            {
                axis: data["groups"][axis]
                for axis in AXES
            },
            ensure_ascii=False,
            separators=(",", ":"),
        ).replace("<", "\\u003c"),
        base_binary=html.escape(timing["baseline"]["binary_sha256"]),
        current_binary=html.escape(timing["current"]["binary_sha256"]),
        suite=html.escape(timing["current"]["suite_sha256"]),
        base_solver=_fmt(timing["baseline"]["solver_time_sum_sec"], 1),
        current_solver=_fmt(timing["current"]["solver_time_sum_sec"], 1),
        solver_ratio=_fmt(timing["solver_sum_ratio"], 2),
    )


def generate_comparison_dashboard(
    baseline_rows_path,
    baseline_timing_path,
    current_rows_path,
    current_timing_path,
    manifest_path,
    out_dir,
    baseline_case_prefix,
    current_case_prefix,
    baseline_label="baseline full",
    current_label="current full",
    report_url=None,
    baseline_dashboard_url=None,
    current_dashboard_url=None,
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
    output = Path(out_dir)
    output.mkdir(parents=True, exist_ok=True)
    page = _generate_html(
        data,
        baseline_label=baseline_label,
        current_label=current_label,
        baseline_rows_url=_relative_url(baseline_rows_path, output),
        current_rows_url=_relative_url(current_rows_path, output),
        report_url=report_url,
        baseline_dashboard_url=baseline_dashboard_url,
        current_dashboard_url=current_dashboard_url,
    )
    (output / "index.html").write_text(page, encoding="utf-8")
    (output / "summary.json").write_text(
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
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--baseline-case-prefix", required=True)
    parser.add_argument("--current-case-prefix", required=True)
    parser.add_argument("--baseline-label", default="baseline full")
    parser.add_argument("--current-label", default="current full")
    parser.add_argument("--report-url")
    parser.add_argument("--baseline-dashboard-url")
    parser.add_argument("--current-dashboard-url")
    args = parser.parse_args()
    data = generate_comparison_dashboard(
        baseline_rows_path=args.baseline_rows,
        baseline_timing_path=args.baseline_timing,
        current_rows_path=args.current_rows,
        current_timing_path=args.current_timing,
        manifest_path=args.manifest,
        out_dir=args.out_dir,
        baseline_case_prefix=args.baseline_case_prefix,
        current_case_prefix=args.current_case_prefix,
        baseline_label=args.baseline_label,
        current_label=args.current_label,
        report_url=args.report_url,
        baseline_dashboard_url=args.baseline_dashboard_url,
        current_dashboard_url=args.current_dashboard_url,
    )
    print(
        "wrote {}: {} cases; solved {}/{} -> {}/{}".format(
            args.out_dir,
            data["overview"]["total"],
            data["overview"]["baseline_solved"],
            data["overview"]["total"],
            data["overview"]["current_solved"],
            data["overview"]["total"],
        )
    )


if __name__ == "__main__":
    main()
