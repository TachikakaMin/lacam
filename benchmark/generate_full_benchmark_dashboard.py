#!/usr/bin/env python3
"""Generate an interactive dashboard from an explicit full-benchmark run."""

import argparse
import csv
import hashlib
import json
import math
import statistics
import subprocess
import sys
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path


BENCH = Path(__file__).resolve().parent
DEFAULT_MANIFEST = (
    BENCH
    / "viz_web"
    / "warehouse_case_proposal"
    / "factorial_suite"
    / "manifest.json"
)
FULL_SUITE = BENCH / "full_benchmark.json"
VIZ_GENERATOR = BENCH / "generate_web_viz.py"


def _success(row):
    return str(row["success"]).lower() in {"1", "true", "yes"}


def _number(row, key):
    value = row.get(key, "")
    return float(value) if value not in ("", None) else None


def _percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    index = min(len(values) - 1, int(math.ceil(fraction * len(values))) - 1)
    return values[index]


def _summary(rows):
    solved = [row for row in rows if _success(row)]
    makespans = [_number(row, "executed_makespan") for row in solved]
    runtimes = [_number(row, "runtime_sec") for row in rows]
    return {
        "total": len(rows),
        "solved": len(solved),
        "failed": len(rows) - len(solved),
        "success_rate": len(solved) / len(rows) if rows else 0,
        "avg_makespan": statistics.mean(makespans) if makespans else None,
        "median_makespan": statistics.median(makespans) if makespans else None,
        "avg_runtime": statistics.mean(runtimes) if runtimes else None,
        "p95_runtime": _percentile(runtimes, 0.95),
        "max_runtime": max(runtimes) if runtimes else None,
    }


def _group_summaries(records, key):
    grouped = defaultdict(list)
    for record in records:
        grouped[str(record[key])].append(record)
    output = []
    for value, items in sorted(grouped.items()):
        makespans = [item["makespan"] for item in items if item["success"]]
        runtimes = [
            item["runtime"] for item in items
            if item["runtime"] is not None
        ]
        output.append(
            {
                "key": value,
                "total": len(items),
                "solved": sum(item["success"] for item in items),
                "avg_makespan": (
                    statistics.mean(makespans) if makespans else None
                ),
                "median_makespan": (
                    statistics.median(makespans) if makespans else None
                ),
                "avg_runtime": (
                    statistics.mean(runtimes) if runtimes else None
                ),
                "p95_runtime": _percentile(runtimes, 0.95),
                "max_runtime": max(runtimes) if runtimes else None,
            }
        )
    return output


def build_dashboard_data(rows_path, timing_path, manifest_path):
    with Path(rows_path).open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    timing = json.loads(Path(timing_path).read_text(encoding="utf-8"))
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    metadata = {record["id"]: record for record in manifest}

    factorial_rows = [row for row in rows if row["instance"] in metadata]
    quick_rows = [row for row in rows if row["instance"] not in metadata]
    all_cases = []
    for row in rows:
        all_cases.append(
            {
                "instance": row["instance"],
                "family": row["family"],
                "success": _success(row),
                "status": row["status"],
                "makespan": _number(row, "executed_makespan"),
                "soc": _number(row, "weighted_soc"),
                "runtime": _number(row, "runtime_sec"),
                "plan_sha256": row["plan_sha256"],
                "animation_url": (
                    "cases/{}.html".format(row["instance"])
                    if _success(row)
                    else None
                ),
            }
        )
    records = []
    for row in factorial_rows:
        meta = metadata[row["instance"]]
        records.append(
            {
                "instance": row["instance"],
                "success": _success(row),
                "makespan": _number(row, "executed_makespan"),
                "soc": _number(row, "weighted_soc"),
                "runtime": _number(row, "runtime_sec"),
                "map_family": meta["map_family"],
                "height": meta["height"],
                "width": meta["width"],
                "block_size": meta["block_size"],
                "density_level": meta["density_level"],
                "agent_level": meta["agent_level"],
                "robots": meta["robots"],
                "targets": meta["targets"],
                "task_profile": meta["task_profile"],
                "goal_mode": meta["goal_mode"],
                "goal_pool_size": meta["goal_pool_size"],
                "animation_url": (
                    "cases/{}.html".format(row["instance"])
                    if _success(row)
                    else None
                ),
                "yaml_url": (
                    "../warehouse_case_proposal/factorial_suite/"
                    + meta["yaml"]
                ),
            }
        )

    failure_groups = defaultdict(int)
    for row in quick_rows:
        if not _success(row):
            failure_groups[(row["family"], row["status"])] += 1

    axes = {}
    for key in (
        "map_family",
        "density_level",
        "agent_level",
        "task_profile",
        "goal_mode",
    ):
        axes[key] = _group_summaries(records, key)

    return {
        "overview": _summary(rows),
        "quick": _summary(quick_rows),
        "factorial": _summary(factorial_rows),
        "timing": {
            "wall_time_sec": timing["wall_time_sec"],
            "solver_time_sum_sec": timing["methods"]["carrier"][
                "solver_time_sum_sec"
            ],
            "jobs": timing["jobs"],
            "timeout_sec": timing["timeout_per_run_sec"],
            "suite_sha256": timing["suite"]["definition_sha256"],
            "binary_sha256": timing["provenance"]["binary_sha256"],
        },
        "axes": axes,
        "failure_groups": [
            {
                "key": "{} · {}".format(family, status),
                "family": family,
                "status": status,
                "count": count,
            }
            for (family, status), count in sorted(failure_groups.items())
        ],
        "slowest": sorted(
            (
                record for record in records
                if record["success"] and record["runtime"] is not None
            ),
            key=lambda item: item["runtime"], reverse=True
        )[:12],
        "longest": sorted(
            (
                record for record in records
                if record["success"] and record["makespan"] is not None
            ),
            key=lambda item: item["makespan"], reverse=True
        )[:12],
        "all_cases": all_cases,
        "cases": records,
    }


def generate_case_animation(case, instance_path, plan_path, output_path):
    instance_path = Path(instance_path)
    plan_path = Path(plan_path)
    output_path = Path(output_path)
    if not plan_path.is_file():
        raise ValueError("missing plan for solved case: {}".format(plan_path))
    digest = hashlib.sha256(plan_path.read_bytes()).hexdigest()
    if digest != case["plan_sha256"]:
        raise ValueError(
            "plan SHA mismatch for {}: {} != {}".format(
                case["instance"], digest, case["plan_sha256"]
            )
        )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    title = "{} · makespan {} · weighted SOC {:.0f}".format(
        case["instance"], case["makespan"], case["soc"]
    )
    proc = subprocess.run(
        [
            sys.executable,
            str(VIZ_GENERATOR),
            str(instance_path),
            str(plan_path),
            str(output_path),
            "--title",
            title,
        ],
        capture_output=True,
        text=True,
        timeout=120,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            "visualization failed for {}: {}".format(
                case["instance"], (proc.stdout + proc.stderr)[-600:]
            )
        )
    return output_path


def _full_instance_map():
    sys.path.insert(0, str(BENCH))
    from run_benchmark import discover_suite_cases

    _, cases, _ = discover_suite_cases(FULL_SUITE)
    return {path.stem: path for path, _ in cases}


def generate_case_animations(data, rows_path, out_dir, jobs=14):
    instances = _full_instance_map()
    work_dir = Path(rows_path).parent / "work"
    cases_dir = Path(out_dir) / "cases"
    solved = [case for case in data["all_cases"] if case["success"]]

    def generate(case):
        name = case["instance"]
        return generate_case_animation(
            case,
            instances[name],
            work_dir / (name + ".carrier.plan"),
            cases_dir / (name + ".html"),
        )

    completed = []
    with ThreadPoolExecutor(max_workers=jobs) as executor:
        futures = {executor.submit(generate, case): case for case in solved}
        for future in as_completed(futures):
            output = future.result()
            completed.append(output.stem)
            print(
                "[{}/{}] {}".format(
                    len(completed), len(solved), output.name
                ),
                flush=True,
            )
    return sorted(completed)


HTML = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Carrier-LaCAM 全量 benchmark 可视化</title>
<style>
:root {
  --bg:#07111f; --panel:#0d1b2d; --panel2:#11243a; --line:#25415f;
  --text:#eef6ff; --muted:#91a7bd; --cyan:#2dd4bf; --blue:#60a5fa;
  --amber:#fbbf24; --red:#fb7185; --violet:#a78bfa;
}
* { box-sizing:border-box; }
body { margin:0; color:var(--text); background:
  radial-gradient(circle at 10% 0%, #173154 0, transparent 32rem),
  radial-gradient(circle at 95% 18%, #153c42 0, transparent 28rem), var(--bg);
  font-family:Inter,ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif; }
main { max-width:1480px; margin:auto; padding:28px; }
a { color:#8ec5ff; }
.eyebrow { color:var(--cyan); letter-spacing:.14em; font-size:12px;
  text-transform:uppercase; font-weight:800; }
h1 { margin:8px 0 8px; font-size:clamp(28px,4vw,52px); line-height:1.05; }
h2 { margin:0 0 14px; font-size:20px; }
p { color:var(--muted); }
.topline { display:flex; gap:18px; align-items:flex-start;
  justify-content:space-between; flex-wrap:wrap; }
.back { padding:9px 13px; border:1px solid var(--line); border-radius:10px;
  text-decoration:none; background:#0b1828cc; }
.cards { display:grid; grid-template-columns:repeat(auto-fit,minmax(180px,1fr));
  gap:14px; margin:24px 0; }
.card,.panel { background:linear-gradient(145deg,#10233aee,#0a1728ee);
  border:1px solid var(--line); border-radius:16px; box-shadow:0 16px 40px #0004; }
.card { padding:18px; }
.card .label { color:var(--muted); font-size:12px; text-transform:uppercase;
  letter-spacing:.08em; }
.card .value { font-size:32px; font-weight:850; margin-top:7px; }
.card .sub { color:var(--muted); font-size:12px; margin-top:4px; }
.grid { display:grid; grid-template-columns:1.15fr .85fr; gap:16px; margin:16px 0; }
.panel { padding:18px; min-width:0; }
.controls { display:flex; flex-wrap:wrap; gap:10px; margin-bottom:14px; }
select,input { color:var(--text); background:#071523; border:1px solid var(--line);
  border-radius:9px; padding:8px 10px; }
.bars { display:grid; gap:10px; }
.bar-row { display:grid; grid-template-columns:110px 1fr 76px; gap:10px;
  align-items:center; font-size:13px; }
.track { height:16px; background:#07121f; border-radius:999px; overflow:hidden; }
.fill { height:100%; border-radius:999px;
  background:linear-gradient(90deg,var(--cyan),var(--blue)); }
.value-right { text-align:right; font-variant-numeric:tabular-nums; }
.legend { display:flex; gap:14px; flex-wrap:wrap; color:var(--muted);
  font-size:12px; margin-top:10px; }
.dot { display:inline-block; width:9px; height:9px; border-radius:50%;
  margin-right:5px; }
svg { width:100%; height:auto; background:#071523; border-radius:12px; }
.tooltip { position:fixed; pointer-events:none; opacity:0; z-index:5;
  background:#020914ee; border:1px solid var(--line); border-radius:9px;
  padding:9px 11px; font-size:12px; max-width:360px; }
.timeout-row { display:grid; grid-template-columns:90px 1fr 40px; gap:8px;
  align-items:center; margin:12px 0; }
.timeout-fill { height:13px; border-radius:99px;
  background:linear-gradient(90deg,var(--amber),var(--red)); }
.callout { border-left:3px solid var(--cyan); padding:8px 12px;
  background:#0b1b2d; color:#c8d8e8; border-radius:0 10px 10px 0; }
.rank { display:grid; grid-template-columns:1fr 70px; gap:7px; margin:8px 0;
  font-size:12px; }
.rank a { overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
.table-wrap { overflow:auto; max-height:620px; border-radius:12px; }
table { width:100%; border-collapse:collapse; font-size:12px; }
th { position:sticky; top:0; background:#10233a; text-align:left; color:#bdd1e5; }
th,td { padding:9px 10px; border-bottom:1px solid #1a324b; white-space:nowrap; }
tbody tr:hover { background:#16304b; }
.pill { padding:3px 7px; border-radius:99px; background:#18314b; }
.footer { margin:20px 0 0; color:var(--muted); font-size:12px; }
@media(max-width:900px) {
  main { padding:18px; } .cards { grid-template-columns:repeat(2,1fr); }
  .grid { grid-template-columns:1fr; }
}
@media(max-width:520px) { .cards { grid-template-columns:1fr; } }
</style>
</head>
<body>
<main>
  <div class="topline">
    <div>
      <div class="eyebrow">Carrier-LaCAM · Full __OVERVIEW_TOTAL__</div>
      <h1>全量 benchmark 可视化</h1>
      <p>本次输入包含 quick __QUICK_TOTAL__ cases + factorial __FACTORIAL_TOTAL__ cases。__TIMEOUT__s/case，__JOBS__ jobs。</p>
    </div>
    <a class="back" href="../index.html">← 返回可视化首页</a>
  </div>

  <section class="cards">
    <div class="card"><div class="label">全量成功率</div>
      <div class="value">__OVERVIEW_SOLVED__/__OVERVIEW_TOTAL__</div>
      <div class="sub">__SUCCESS_PERCENT__% solved</div></div>
    <div class="card"><div class="label">新增 factorial</div>
      <div class="value" style="color:var(--cyan)">__FACTORIAL_SOLVED__/__FACTORIAL_TOTAL__</div>
      <div class="sub">__FACTORIAL_FAILED__ failed</div></div>
    <div class="card"><div class="label">原 quick</div>
      <div class="value">__QUICK_SOLVED__/__QUICK_TOTAL__</div>
      <div class="sub">__QUICK_FAILED__ failed</div></div>
    <div class="card"><div class="label">全量 wall time</div>
      <div class="value">__WALL_TIME__s</div>
      <div class="sub">solver sum __SOLVER_SUM__s · jobs=__JOBS__</div></div>
    <div class="card"><div class="label">实际轨迹动画</div>
      <div class="value" style="color:var(--blue)">__OVERVIEW_SOLVED__</div>
      <div class="sub">仅由本轮成功 plan 生成</div></div>
  </section>

  <section class="grid">
    <div class="panel">
      <h2>因素趋势</h2>
      <div class="controls">
        <select id="axisDimension">
          <option value="map_family">地图族</option>
          <option value="density_level">货架密度</option>
          <option value="agent_level">Agent 数量</option>
          <option value="task_profile">任务分布</option>
          <option value="goal_mode">Goal 模式</option>
        </select>
        <select id="axisMetric">
          <option value="avg_makespan">平均 makespan</option>
          <option value="avg_runtime">平均运行时间</option>
          <option value="p95_runtime">P95 运行时间</option>
        </select>
      </div>
      <div id="axisBars" class="bars"></div>
      <div class="callout" id="axisInsight"></div>
    </div>
    <div class="panel">
      <h2>原 quick 的 __QUICK_FAILED__ 个失败</h2>
      <div id="timeoutBars"></div>
      <p>失败按 family 与原始 status 分组：__FAILURE_SUMMARY__</p>
    </div>
  </section>

  <section class="panel">
    <h2>新增 __FACTORIAL_TOTAL__ cases：makespan × runtime</h2>
    <div class="controls" id="scatterControls"></div>
    <svg id="caseScatter" viewBox="0 0 1000 470" role="img"
      aria-label="factorial cases makespan runtime scatter plot"></svg>
    <div id="scatterLegend" class="legend"></div>
  </section>

  <section class="grid">
    <div class="panel"><h2>运行时间最慢</h2><div id="slowest"></div></div>
    <div class="panel"><h2>Makespan 最长</h2><div id="longest"></div></div>
  </section>

  <section class="panel">
    <div class="topline">
      <h2>Factorial case 明细</h2>
      <input id="caseSearch" placeholder="搜索 instance…" size="28">
    </div>
    <div class="table-wrap">
      <table><thead><tr>
        <th>Instance</th><th>地图</th><th>密度</th><th>Agent</th>
        <th>任务</th><th>Goal</th><th>Robots</th><th>Targets</th>
        <th>Makespan</th><th>SOC</th><th>Runtime</th><th>YAML</th>
      </tr></thead><tbody id="caseRows"></tbody></table>
    </div>
  </section>

  <section class="panel">
    <div class="topline">
      <div>
        <h2>全部 __OVERVIEW_TOTAL__ 个实际 benchmark 结果</h2>
        <p>成功案例可播放本轮 rows.csv 对应、SHA 校验后的真实计划；失败案例保留 runner 的原始 status。</p>
      </div>
      <div class="controls">
        <select id="allFamily"><option value="">全部 family</option></select>
        <select id="allStatus"><option value="">全部状态</option></select>
        <input id="allSearch" placeholder="搜索全部 instance…" size="25">
      </div>
    </div>
    <div class="table-wrap">
      <table><thead><tr><th>Instance</th><th>Family</th><th>Status</th>
        <th>Makespan</th><th>SOC</th><th>Runtime</th><th>实际动画</th>
      </tr></thead><tbody id="allRows"></tbody></table>
    </div>
  </section>
  <div class="footer">Suite SHA: <span id="suiteSha"></span> · Binary SHA: <span id="binarySha"></span></div>
</main>
<div class="tooltip" id="tooltip"></div>
<script>
const DATA = __DATA__;
const COLORS = {g1:"#2dd4bf",g2:"#38bdf8",g3:"#60a5fa",g4:"#818cf8",g5:"#a78bfa",g6:"#f472b6"};
const LABELS = {
  low:"低",medium:"中",high:"高",scarce:"scarce",baseline:"baseline",
  equal:"equal",surplus:"surplus",local:"local",mixed:"mixed",
  cross_heavy:"cross-heavy",singleton:"singleton",shared_pool:"shared pool"
};
const fmt = (v,d=1) => Number(v).toFixed(d);

function renderAxis() {
  const dim=document.querySelector("#axisDimension").value;
  const metric=document.querySelector("#axisMetric").value;
  const rows=DATA.axes[dim];
  const max=Math.max(...rows.map(r=>r[metric]));
  const suffix=metric.includes("runtime")?"s":"";
  document.querySelector("#axisBars").innerHTML=rows.map(r=>`
    <div class="bar-row"><span>${LABELS[r.key]||r.key}</span>
      <div class="track"><div class="fill" style="width:${100*r[metric]/max}%"></div></div>
      <span class="value-right">${fmt(r[metric],metric.includes("runtime")?3:1)}${suffix}</span>
    </div>`).join("");
  const lo=rows.reduce((a,b)=>a[metric]<b[metric]?a:b);
  const hi=rows.reduce((a,b)=>a[metric]>b[metric]?a:b);
  document.querySelector("#axisInsight").textContent =
    `${LABELS[lo.key]||lo.key} 最低，${LABELS[hi.key]||hi.key} 最高；差值 ${fmt(hi[metric]-lo[metric],metric.includes("runtime")?3:1)}${suffix}。`;
}

function renderFailures() {
  if (!DATA.failure_groups.length) {
    document.querySelector("#timeoutBars").textContent="本轮 quick 无失败";
    return;
  }
  const max=Math.max(...DATA.failure_groups.map(x=>x.count));
  document.querySelector("#timeoutBars").innerHTML=DATA.failure_groups.map(x=>`
    <div class="timeout-row"><span>${x.key}</span><div class="track">
      <div class="timeout-fill" style="width:${100*x.count/max}%"></div></div>
      <b>${x.count}</b></div>`).join("");
}

const dims=["density_level","agent_level","task_profile","goal_mode"];
function buildScatterControls() {
  const root=document.querySelector("#scatterControls");
  root.innerHTML=dims.map(dim=>{
    const values=[...new Set(DATA.cases.map(x=>x[dim]))].sort();
    return `<select data-filter="${dim}"><option value="">${dim}: 全部</option>`+
      values.map(v=>`<option value="${v}">${LABELS[v]||v}</option>`).join("")+`</select>`;
  }).join("");
  root.querySelectorAll("select").forEach(x=>x.addEventListener("change",renderScatter));
}

function selectedCases() {
  const filters={};
  document.querySelectorAll("[data-filter]").forEach(x=>filters[x.dataset.filter]=x.value);
  return DATA.cases.filter(c=>c.success && dims.every(dim=>!filters[dim]||c[dim]===filters[dim]));
}

function renderScatter() {
  const cases=selectedCases(), svg=document.querySelector("#caseScatter");
  const W=1000,H=470,L=72,R=28,T=24,B=52;
  const maxX=Math.max(10,...cases.map(c=>c.makespan))*1.05;
  const minY=Math.min(...cases.map(c=>c.runtime));
  const maxY=Math.max(...cases.map(c=>c.runtime));
  const logMin=Math.log10(Math.max(.005,minY*.8)), logMax=Math.log10(maxY*1.2);
  const x=v=>L+(W-L-R)*v/maxX;
  const y=v=>T+(H-T-B)*(1-(Math.log10(v)-logMin)/(logMax-logMin||1));
  let html=`<rect x="${L}" y="${T}" width="${W-L-R}" height="${H-T-B}" fill="#071523"/>`;
  [0,.25,.5,.75,1].forEach(f=>{
    const xx=L+(W-L-R)*f;
    html+=`<line x1="${xx}" y1="${T}" x2="${xx}" y2="${H-B}" stroke="#203b56"/>`;
    html+=`<text x="${xx}" y="${H-20}" fill="#91a7bd" text-anchor="middle" font-size="12">${Math.round(maxX*f)}</text>`;
  });
  [0,.25,.5,.75,1].forEach(f=>{
    const yy=T+(H-T-B)*f, val=Math.pow(10,logMax-(logMax-logMin)*f);
    html+=`<line x1="${L}" y1="${yy}" x2="${W-R}" y2="${yy}" stroke="#203b56"/>`;
    html+=`<text x="${L-10}" y="${yy+4}" fill="#91a7bd" text-anchor="end" font-size="12">${fmt(val,val<.1?2:1)}s</text>`;
  });
  html+=`<text x="${(L+W-R)/2}" y="${H-3}" fill="#bdd1e5" text-anchor="middle">executed makespan</text>`;
  html+=`<text x="15" y="${H/2}" fill="#bdd1e5" transform="rotate(-90 15 ${H/2})" text-anchor="middle">runtime（log scale）</text>`;
  html+=cases.map((c,i)=>`<circle cx="${x(c.makespan)}" cy="${y(c.runtime)}" r="5"
    fill="${COLORS[c.map_family]}" fill-opacity=".75" stroke="#fff" stroke-opacity=".18"
    data-case="${DATA.cases.indexOf(c)}"></circle>`).join("");
  svg.innerHTML=html;
  const tip=document.querySelector("#tooltip");
  svg.querySelectorAll("circle").forEach(dot=>{
    dot.addEventListener("mousemove",e=>{
      const c=DATA.cases[Number(dot.dataset.case)];
      tip.innerHTML=`<b>${c.instance}</b><br>makespan ${c.makespan} · runtime ${fmt(c.runtime,3)}s<br>${c.map_family} · ${c.density_level} · ${c.agent_level} · ${c.task_profile} · ${c.goal_mode}`;
      tip.style.left=(e.clientX+14)+"px"; tip.style.top=(e.clientY+14)+"px"; tip.style.opacity=1;
    });
    dot.addEventListener("mouseleave",()=>tip.style.opacity=0);
  });
}

function renderRanks(id,rows,metric,suffix) {
  document.querySelector("#"+id).innerHTML=rows.map(c=>`
    <div class="rank"><a href="${c.animation_url}" title="${c.instance}">${c.instance}</a>
    <span class="value-right">${fmt(c[metric],metric==="runtime"?3:0)}${suffix}</span></div>`).join("");
}

function renderTable() {
  const q=document.querySelector("#caseSearch").value.toLowerCase();
  const rows=DATA.cases.filter(c=>c.instance.toLowerCase().includes(q));
  document.querySelector("#caseRows").innerHTML=rows.map(c=>`<tr>
    <td>${c.success?`<a href="${c.animation_url}">▶ ${c.instance}</a>`:c.instance}</td><td><span class="pill">${c.map_family}</span></td>
    <td>${c.density_level}</td><td>${c.agent_level}</td><td>${c.task_profile}</td>
    <td>${c.goal_mode}</td><td>${c.robots}</td><td>${c.targets}</td>
    <td>${c.makespan===null?"—":c.makespan}</td><td>${c.soc===null?"—":fmt(c.soc,0)}</td><td>${fmt(c.runtime,3)}s</td>
    <td><a href="${c.yaml_url}">YAML</a></td></tr>`).join("");
}

function renderAllCases() {
  const family=document.querySelector("#allFamily").value;
  const status=document.querySelector("#allStatus").value;
  const q=document.querySelector("#allSearch").value.toLowerCase();
  const rows=DATA.all_cases.filter(c=>
    (!family||c.family===family) &&
    (!status||(status==="solved" ? c.success : (!c.success && c.status===status))) &&
    c.instance.toLowerCase().includes(q));
  document.querySelector("#allRows").innerHTML=rows.map(c=>`<tr>
    <td>${c.success?`<a href="${c.animation_url}">▶ ${c.instance}</a>`:c.instance}</td>
    <td>${c.family}</td><td>${c.success?`<span style="color:var(--cyan)">solved</span>`:`<span style="color:var(--red)">${c.status}</span>`}</td>
    <td>${c.makespan===null?"—":c.makespan}</td><td>${c.soc===null?"—":fmt(c.soc,0)}</td>
    <td>${fmt(c.runtime,3)}s</td><td>${c.success?`<a href="${c.animation_url}">播放真实计划</a>`:"—"}</td>
  </tr>`).join("");
}

document.querySelector("#axisDimension").addEventListener("change",renderAxis);
document.querySelector("#axisMetric").addEventListener("change",renderAxis);
document.querySelector("#caseSearch").addEventListener("input",renderTable);
const allFamilies=[...new Set(DATA.all_cases.map(c=>c.family))].sort();
document.querySelector("#allFamily").innerHTML += allFamilies.map(
  family=>`<option value="${family}">${family}</option>`).join("");
const allStatuses=[...new Set(DATA.all_cases.filter(c=>!c.success).map(c=>c.status))].sort();
document.querySelector("#allStatus").innerHTML +=
  `<option value="solved">solved</option>`+
  allStatuses.map(status=>`<option value="${status}">${status}</option>`).join("");
document.querySelector("#allFamily").addEventListener("change",renderAllCases);
document.querySelector("#allStatus").addEventListener("change",renderAllCases);
document.querySelector("#allSearch").addEventListener("input",renderAllCases);
document.querySelector("#suiteSha").textContent=DATA.timing.suite_sha256.slice(0,16)+"…";
document.querySelector("#binarySha").textContent=DATA.timing.binary_sha256.slice(0,16)+"…";
renderAxis(); renderFailures(); buildScatterControls(); renderScatter();
renderRanks("slowest",DATA.slowest,"runtime","s");
renderRanks("longest",DATA.longest,"makespan","");
renderTable(); renderAllCases();
document.querySelector("#scatterLegend").innerHTML=Object.entries(COLORS).map(([k,v])=>
  `<span><i class="dot" style="background:${v}"></i>${k}</span>`).join("");
</script>
</body>
</html>
"""


def generate_dashboard(rows_path, timing_path, manifest_path, out_dir):
    data = build_dashboard_data(rows_path, timing_path, manifest_path)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    encoded = encoded.replace("</", "<\\/")
    failure_summary = "；".join(
        "{}×{}".format(group["key"], group["count"])
        for group in data["failure_groups"]
    ) or "无"
    replacements = {
        "__DATA__": encoded,
        "__OVERVIEW_TOTAL__": str(data["overview"]["total"]),
        "__OVERVIEW_SOLVED__": str(data["overview"]["solved"]),
        "__SUCCESS_PERCENT__": "{:.1f}".format(
            100 * data["overview"]["success_rate"]
        ),
        "__QUICK_TOTAL__": str(data["quick"]["total"]),
        "__QUICK_SOLVED__": str(data["quick"]["solved"]),
        "__QUICK_FAILED__": str(data["quick"]["failed"]),
        "__FACTORIAL_TOTAL__": str(data["factorial"]["total"]),
        "__FACTORIAL_SOLVED__": str(data["factorial"]["solved"]),
        "__FACTORIAL_FAILED__": str(data["factorial"]["failed"]),
        "__WALL_TIME__": "{:.1f}".format(
            data["timing"]["wall_time_sec"]
        ),
        "__SOLVER_SUM__": "{:.1f}".format(
            data["timing"]["solver_time_sum_sec"]
        ),
        "__JOBS__": str(data["timing"]["jobs"]),
        "__TIMEOUT__": "{:g}".format(data["timing"]["timeout_sec"]),
        "__FAILURE_SUMMARY__": failure_summary,
    }
    page = HTML
    for token, value in replacements.items():
        page = page.replace(token, value)
    (out_dir / "index.html").write_text(page, encoding="utf-8")
    (out_dir / "summary.json").write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rows", type=Path, required=True)
    parser.add_argument("--timing", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=14)
    parser.add_argument("--skip-cases", action="store_true")
    args = parser.parse_args()
    data = generate_dashboard(
        args.rows, args.timing, args.manifest, args.out_dir
    )
    if not args.skip_cases:
        generated = generate_case_animations(
            data, args.rows, args.out_dir, jobs=args.jobs
        )
        if len(generated) != data["overview"]["solved"]:
            raise RuntimeError(
                "generated {} animations for {} solved cases".format(
                    len(generated), data["overview"]["solved"]
                )
            )
    print(
        "wrote {}: {}/{} solved; factorial {}/{}; animations={}".format(
            args.out_dir,
            data["overview"]["solved"],
            data["overview"]["total"],
            data["factorial"]["solved"],
            data["factorial"]["total"],
            data["overview"]["solved"] if not args.skip_cases else "skipped",
        )
    )


if __name__ == "__main__":
    main()
