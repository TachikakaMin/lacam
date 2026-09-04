#!/usr/bin/env python3
"""Generate a self-contained interactive small-map quality dashboard."""

import csv
import json
import math
import re
from pathlib import Path


BENCH = Path(__file__).resolve().parent
BASE_CSV = BENCH / "results_v3_strict_return_final" / "rows.csv"
NEW_CSV = BENCH / "results_v4_1_final7" / "rows.csv"
OUTPUT = BENCH / "viz_web" / "small_quality_final7.html"


def load(path):
    with path.open(newline="") as stream:
        return {row["instance"]: row for row in csv.DictReader(stream)}


def parse_name(name):
    match = re.search(
        r"brap_h(\d+)w(\d+)_a(\d+)_e(\d+)_(B|R1)_seed(\d+)", name
    )
    if match is None:
        raise ValueError("unexpected instance name: " + name)
    height, width, agents, empties, kind, seed = match.groups()
    return {
        "h": int(height),
        "w": int(width),
        "agents": int(agents),
        "empties": int(empties),
        "kind": kind,
        "seed": int(seed),
    }


def number(row, field):
    value = row.get(field, "")
    return float(value) if value not in ("", None) else None


def geometric_mean(values):
    return math.exp(sum(math.log(value) for value in values) / len(values))


def summarize(records):
    common = [record for record in records if record["base_ok"] and record["new_ok"]]
    mk_ratios = [record["mk_ratio"] for record in common if record["mk_ratio"]]
    soc_ratios = [record["soc_ratio"] for record in common if record["soc_ratio"]]

    def wtl(metric):
        wins = sum(record[metric] < 1 for record in common if record[metric])
        ties = sum(record[metric] == 1 for record in common if record[metric])
        losses = sum(record[metric] > 1 for record in common if record[metric])
        zero_ties = sum(
            record[metric] is None
            and record["base_soc"] == 0
            and record["new_soc"] == 0
            for record in common
        )
        return [wins, ties + zero_ties, losses]

    return {
        "total": len(records),
        "base_solved": sum(record["base_ok"] for record in records),
        "new_solved": sum(record["new_ok"] for record in records),
        "common": len(common),
        "mk_ratio": geometric_mean(mk_ratios),
        "soc_ratio": geometric_mean(soc_ratios),
        "mk_wtl": wtl("mk_ratio"),
        "soc_wtl": wtl("soc_ratio"),
    }


def build_data():
    base = load(BASE_CSV)
    new = load(NEW_CSV)
    records = []
    for name in sorted(new):
        meta = parse_name(name)
        if meta["h"] > 10:
            continue
        base_row = base[name]
        new_row = new[name]
        base_ok = base_row["success"] == "1"
        new_ok = new_row["success"] == "1"
        base_mk = number(base_row, "executed_makespan") if base_ok else None
        new_mk = number(new_row, "executed_makespan") if new_ok else None
        base_soc = number(base_row, "weighted_soc") if base_ok else None
        new_soc = number(new_row, "weighted_soc") if new_ok else None
        record = {
            "instance": name,
            "short": "h{h}-a{agents}-e{empties}-{kind}-s{seed}".format(**meta),
            **meta,
            "base_ok": base_ok,
            "new_ok": new_ok,
            "base_mk": base_mk,
            "new_mk": new_mk,
            "base_soc": base_soc,
            "new_soc": new_soc,
            "mk_ratio": (
                new_mk / base_mk
                if base_ok and new_ok and base_mk and new_mk
                else None
            ),
            "soc_ratio": (
                new_soc / base_soc
                if base_ok and new_ok and base_soc and new_soc
                else None
            ),
            "runtime": number(new_row, "runtime_sec"),
            "deliverable_ms": number(new_row, "deliverable_ms"),
        }
        records.append(record)

    groups = []
    for label, selected in [
        ("h4", [record for record in records if record["h"] == 4]),
        ("h6", [record for record in records if record["h"] == 6]),
        ("h8", [record for record in records if record["h"] == 8]),
        ("h10", [record for record in records if record["h"] == 10]),
        ("B / pool", [record for record in records if record["kind"] == "B"]),
        ("R1", [record for record in records if record["kind"] == "R1"]),
    ]:
        groups.append({"label": label, **summarize(selected)})

    return {
        "generated": "2026-09-02",
        "protocol": "strict 10s · seed 0 · jobs 14 · unit weights · following",
        "records": records,
        "overall": summarize(records),
        "groups": groups,
        "new_only": [
            record for record in records if record["new_ok"] and not record["base_ok"]
        ],
    }


HTML = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Carrier-LaCAM 小图质量分析 · final7</title>
<style>
:root {
  --bg:#090d18; --panel:#111827; --panel2:#172033; --line:#2b3850;
  --text:#e7edf7; --muted:#94a3b8; --blue:#4da3ff; --blue2:#2563eb;
  --green:#35c98d; --red:#ff6b6b; --amber:#f7b955; --purple:#a78bfa;
}
* { box-sizing:border-box; }
body {
  margin:0; color:var(--text); background:
    radial-gradient(circle at 15% 0%, #152449 0, transparent 31rem),
    radial-gradient(circle at 95% 5%, #2b163d 0, transparent 29rem), var(--bg);
  font-family:-apple-system,BlinkMacSystemFont,"Segoe UI","Noto Sans SC",sans-serif;
}
.page { max-width:1560px; margin:auto; padding:28px; }
h1 { margin:0; font-size:29px; letter-spacing:-.02em; }
h2 { margin:0 0 16px; font-size:18px; }
.sub { color:var(--muted); margin:8px 0 22px; }
.cards { display:grid; grid-template-columns:repeat(6,minmax(150px,1fr)); gap:12px; }
.card,.panel {
  background:linear-gradient(145deg,rgba(23,32,51,.96),rgba(14,22,37,.96));
  border:1px solid var(--line); border-radius:14px; box-shadow:0 16px 45px #0005;
}
.card { padding:16px; min-height:106px; }
.card .label { color:var(--muted); font-size:12px; text-transform:uppercase; letter-spacing:.08em; }
.card .value { font-size:28px; font-weight:750; margin:8px 0 3px; }
.card .detail { color:var(--muted); font-size:12px; line-height:1.45; }
.good { color:var(--green)!important; } .bad { color:var(--red)!important; }
.warn { color:var(--amber)!important; }
.controls {
  display:flex; gap:10px; flex-wrap:wrap; align-items:center; margin:20px 0 14px;
  padding:12px 14px; background:#0f1728dd; border:1px solid var(--line); border-radius:12px;
}
label { color:var(--muted); font-size:13px; }
select,input {
  color:var(--text); background:#172033; border:1px solid #34435d;
  border-radius:7px; padding:7px 9px; margin-left:5px;
}
input { width:220px; }
.grid { display:grid; grid-template-columns:1fr 1fr; gap:14px; }
.panel { padding:18px; min-width:0; }
.wide { grid-column:1/-1; }
.note { color:var(--muted); font-size:12px; margin-top:-9px; margin-bottom:12px; }
.coverage-row,.quality-row {
  display:grid; grid-template-columns:72px 1fr 78px; gap:10px; align-items:center; margin:12px 0;
}
.track { height:25px; background:#0b1220; border:1px solid #29364c; border-radius:7px; position:relative; overflow:hidden; }
.coverage-base,.coverage-new { position:absolute; left:0; border-radius:5px; }
.coverage-base { height:8px; top:3px; background:#778399; }
.coverage-new { height:8px; bottom:3px; background:var(--blue); }
.quality-track { height:22px; background:#0b1220; border:1px solid #29364c; border-radius:6px; position:relative; }
.quality-track .one { position:absolute; top:-3px; bottom:-3px; width:2px; background:#fff9; }
.quality-track .bar { position:absolute; top:4px; height:12px; border-radius:4px; min-width:2px; }
.ratio { font-variant-numeric:tabular-nums; text-align:right; font-weight:650; }
.legend { display:flex; gap:17px; color:var(--muted); font-size:12px; margin-top:12px; }
.swatch { width:10px; height:10px; border-radius:2px; display:inline-block; margin-right:5px; }
svg { width:100%; height:auto; min-height:420px; display:block; }
.axis { stroke:#55647c; stroke-width:1; } .gridline { stroke:#253249; stroke-width:1; }
.diag { stroke:#d8e1ef99; stroke-width:1.5; stroke-dasharray:7 5; }
.tick { fill:#9aa8bd; font-size:11px; }
.point { stroke:#07101e; stroke-width:1.2; cursor:pointer; transition:.12s; }
.point:hover { stroke:white; stroke-width:2.2; filter:brightness(1.25); }
.tooltip {
  position:fixed; display:none; z-index:20; pointer-events:none; padding:10px 12px;
  background:#050914f2; border:1px solid #4a5c78; border-radius:8px; font-size:12px;
  box-shadow:0 12px 30px #000a; line-height:1.5;
}
.new-only { display:grid; grid-template-columns:repeat(3,1fr); gap:10px; }
.case-mini { background:#0b1323; border:1px solid #2c3b52; border-radius:9px; padding:12px; }
.case-mini b { color:var(--blue); }
.case-mini div { color:var(--muted); font-size:12px; margin-top:6px; }
.table-wrap { overflow:auto; max-height:760px; border:1px solid #29364c; border-radius:9px; }
table { border-collapse:collapse; width:100%; min-width:1080px; font-size:12px; }
th { position:sticky; top:0; background:#182238; color:#b9c6d8; z-index:2; text-align:left; }
th,td { padding:9px 10px; border-bottom:1px solid #253249; white-space:nowrap; }
tbody tr:hover { background:#1a2740; }
.badge { border-radius:999px; padding:2px 7px; font-weight:650; font-size:11px; }
.ok { color:#9ce8ca; background:#154f3c; } .timeout { color:#ffc3c3; background:#672b36; }
.ratio-pill { min-width:62px; display:inline-block; text-align:center; border-radius:6px; padding:3px 6px; font-weight:700; }
.section-gap { margin-top:14px; }
@media(max-width:1100px) { .cards { grid-template-columns:repeat(3,1fr); } .grid { grid-template-columns:1fr; } .wide { grid-column:auto; } }
@media(max-width:650px) { .page { padding:16px; } .cards,.new-only { grid-template-columns:1fr; } }
</style>
</head>
<body>
<div class="page">
  <h1>Carrier-LaCAM 小图 benchmark：覆盖率与质量</h1>
  <div class="sub">final7 对比 clean-HEAD strict v3 · <span id="protocol"></span> · 比值 &gt; 1 表示变差 · <a href="small_quality_cases.html" style="color:#76b9ff">查看具体 testcase 动画</a></div>
  <div class="cards" id="cards"></div>

  <div class="controls">
    <label>指标<select id="metric"><option value="mk">Makespan</option><option value="soc">Weighted SOC</option></select></label>
    <label>尺寸<select id="height"><option value="all">全部</option><option value="4">h4</option><option value="6">h6</option><option value="8">h8</option><option value="10">h10</option></select></label>
    <label>类型<select id="kind"><option value="all">全部</option><option value="B">B / pool</option><option value="R1">R1</option></select></label>
    <label>排序<select id="sort"><option value="ratio">退化最大</option><option value="improve">改善最大</option><option value="name">实例名</option></select></label>
    <label>搜索<input id="search" placeholder="例如 h8 / e20 / R1"></label>
    <span class="note" id="filterSummary" style="margin:0 0 0 auto"></span>
  </div>

  <div class="grid">
    <section class="panel">
      <h2>A. 小图解出率</h2>
      <div class="note">灰色=v3，蓝色=final7；每组上限为该尺寸实例总数。</div>
      <div id="coverage"></div>
      <div class="legend"><span><i class="swatch" style="background:#778399"></i>strict v3</span><span><i class="swatch" style="background:#4da3ff"></i>final7</span></div>
    </section>
    <section class="panel">
      <h2>B. 分组质量几何均值</h2>
      <div class="note">虚线位置为 1×；绿色改善，红色退化。指标可在上方切换。</div>
      <div id="qualityGroups"></div>
    </section>
    <section class="panel wide">
      <h2>C. 基线 vs final7（共同成功集）</h2>
      <div class="note">对数坐标；对角线上方为退化。圆=B/pool，三角=R1；悬停查看实例。</div>
      <svg id="scatter" viewBox="0 0 1300 480" role="img"></svg>
    </section>
    <section class="panel wide">
      <h2>D. 新增解决的 3 个实例</h2>
      <div class="new-only" id="newOnly"></div>
    </section>
    <section class="panel wide">
      <h2>E. 逐例质量</h2>
      <div class="note">共同成功例显示 final7/v3 比值；new-only 无可比基线。点击筛选器可聚焦 R1 或特定尺寸。</div>
      <div class="table-wrap"><table>
        <thead><tr><th>实例</th><th>尺寸</th><th>类型</th><th>v3</th><th>final7</th><th>mk v3→new</th><th>mk ratio</th><th>SOC v3→new</th><th>SOC ratio</th><th>runtime</th></tr></thead>
        <tbody id="caseRows"></tbody>
      </table></div>
    </section>
  </div>
</div>
<div class="tooltip" id="tooltip"></div>
<script>
const DATA = __DATA__;
const $ = id => document.getElementById(id);
const fmt = (x,d=3) => x == null ? "—" : Number(x).toLocaleString(undefined,{maximumFractionDigits:d});
const status = ok => `<span class="badge ${ok?'ok':'timeout'}">${ok?'solved':'timeout'}</span>`;
const ratioColor = x => x == null ? "#334155" : x <= 1 ? "#35c98d" : "#ff6b6b";
const ratioBg = x => x == null ? "#263247" : x <= 1 ? "#164e3d" : "#672b36";
const ratioText = x => x == null ? "new-only" : `${x.toFixed(3)}×`;

$("protocol").textContent = DATA.protocol;

function cards() {
  const o=DATA.overall, r1=DATA.groups.find(x=>x.label==="R1");
  const defs=[
    ["final7 solved",`${o.new_solved}/${o.total}`,"全 36 个小图解出","good"],
    ["strict v3 solved",`${o.base_solved}/${o.total}`,"基线少 3 例",""],
    ["coverage gain",`+${o.new_solved-o.base_solved}`,"无 baseline-only","good"],
    ["common mk",`${o.mk_ratio.toFixed(3)}×`,`${o.mk_wtl[0]}/${o.mk_wtl[1]}/${o.mk_wtl[2]} 改善/平/退化`,"bad"],
    ["common SOC",`${o.soc_ratio.toFixed(3)}×`,`${o.soc_wtl[0]}/${o.soc_wtl[1]}/${o.soc_wtl[2]} 改善/平/退化`,"bad"],
    ["R1 quality",`${r1.mk_ratio.toFixed(3)}×`, `SOC ${r1.soc_ratio.toFixed(3)}× · 主要风险`,"bad"]
  ];
  $("cards").innerHTML=defs.map(([l,v,d,c])=>`<div class="card"><div class="label">${l}</div><div class="value ${c}">${v}</div><div class="detail">${d}</div></div>`).join("");
}

function coverage() {
  const hs=[4,6,8,10];
  $("coverage").innerHTML=hs.map(h=>{
    const rs=DATA.records.filter(r=>r.h===h), total=rs.length;
    const b=rs.filter(r=>r.base_ok).length, n=rs.filter(r=>r.new_ok).length;
    return `<div class="coverage-row"><b>h${h}</b><div class="track"><div class="coverage-base" style="width:${100*b/total}%"></div><div class="coverage-new" style="width:${100*n/total}%"></div></div><span class="ratio">${b} → ${n}/${total}</span></div>`;
  }).join("");
}

function qualityGroups() {
  const metric=$("metric").value, key=metric+"_ratio";
  const min=.85,max=1.36,pos=x=>100*(x-min)/(max-min), one=pos(1);
  $("qualityGroups").innerHTML=DATA.groups.map(g=>{
    const x=g[key], p=pos(x), left=Math.min(one,p), width=Math.max(1,Math.abs(p-one));
    return `<div class="quality-row"><b>${g.label}</b><div class="quality-track"><span class="one" style="left:${one}%"></span><span class="bar" style="left:${left}%;width:${width}%;background:${ratioColor(x)}"></span></div><span class="ratio" style="color:${ratioColor(x)}">${x.toFixed(3)}×</span></div>`;
  }).join("");
}

function filtered() {
  const h=$("height").value,k=$("kind").value,q=$("search").value.trim().toLowerCase();
  let rs=DATA.records.filter(r=>(h==="all"||r.h===+h)&&(k==="all"||r.kind===k)&&(!q||r.short.toLowerCase().includes(q)||r.instance.toLowerCase().includes(q)));
  const metric=$("metric").value, key=metric+"_ratio", sort=$("sort").value;
  rs.sort((a,b)=>sort==="name"?a.short.localeCompare(b.short):sort==="improve"?(a[key]??99)-(b[key]??99):(b[key]??-1)-(a[key]??-1));
  return rs;
}

function scatter(records) {
  const svg=$("scatter"), metric=$("metric").value;
  const baseKey=metric==="mk"?"base_mk":"base_soc", newKey=metric==="mk"?"new_mk":"new_soc", ratioKey=metric+"_ratio";
  const pts=records.filter(r=>r.base_ok&&r.new_ok&&r[baseKey]>0&&r[newKey]>0);
  if(!pts.length) {
    svg.innerHTML=`<text x="650" y="235" text-anchor="middle" fill="#94a3b8" font-size="16">当前筛选没有共同成功且非零的可比实例</text>`;
    return;
  }
  const W=1300,H=480,m={l:72,r:28,t:18,b:52};
  const vals=pts.flatMap(r=>[r[baseKey],r[newKey]]), lo=Math.log10(Math.min(...vals)*.82), hi=Math.log10(Math.max(...vals)*1.22);
  const X=v=>m.l+(Math.log10(v)-lo)/(hi-lo)*(W-m.l-m.r), Y=v=>H-m.b-(Math.log10(v)-lo)/(hi-lo)*(H-m.t-m.b);
  const powers=[]; for(let p=Math.floor(lo);p<=Math.ceil(hi);p++) powers.push(10**p);
  let html="";
  for(const t of powers){ if(Math.log10(t)<lo||Math.log10(t)>hi) continue; html+=`<line class="gridline" x1="${X(t)}" x2="${X(t)}" y1="${m.t}" y2="${H-m.b}"/><line class="gridline" x1="${m.l}" x2="${W-m.r}" y1="${Y(t)}" y2="${Y(t)}"/><text class="tick" x="${X(t)}" y="${H-m.b+20}" text-anchor="middle">${fmt(t,0)}</text><text class="tick" x="${m.l-10}" y="${Y(t)+4}" text-anchor="end">${fmt(t,0)}</text>`; }
  html+=`<line class="axis" x1="${m.l}" x2="${W-m.r}" y1="${H-m.b}" y2="${H-m.b}"/><line class="axis" x1="${m.l}" x2="${m.l}" y1="${m.t}" y2="${H-m.b}"/><line class="diag" x1="${X(10**lo)}" y1="${Y(10**lo)}" x2="${X(10**hi)}" y2="${Y(10**hi)}"/><text class="tick" x="${(m.l+W-m.r)/2}" y="${H-12}" text-anchor="middle">strict v3 ${metric==="mk"?"makespan":"weighted SOC"}</text><text class="tick" transform="translate(17 ${(m.t+H-m.b)/2}) rotate(-90)" text-anchor="middle">final7</text>`;
  for(const r of pts){
    const x=X(r[baseKey]),y=Y(r[newKey]),color=ratioColor(r[ratioKey]);
    if(r.kind==="R1") html+=`<path class="point" data-id="${r.instance}" fill="${color}" d="M ${x} ${y-7} L ${x-7} ${y+6} L ${x+7} ${y+6} Z"/>`;
    else html+=`<circle class="point" data-id="${r.instance}" cx="${x}" cy="${y}" r="6.5" fill="${color}"/>`;
  }
  svg.innerHTML=html;
  svg.querySelectorAll(".point").forEach(el=>{
    el.onmousemove=e=>showTip(e,DATA.records.find(r=>r.instance===el.dataset.id));
    el.onmouseleave=hideTip;
  });
}

function showTip(e,r) {
  const metric=$("metric").value, ratio=r[metric+"_ratio"];
  $("tooltip").innerHTML=`<b>${r.short}</b><br>type=${r.kind} · h=${r.h} · e=${r.empties}<br>mk ${fmt(r.base_mk,0)} → ${fmt(r.new_mk,0)} (${ratioText(r.mk_ratio)})<br>SOC ${fmt(r.base_soc,0)} → ${fmt(r.new_soc,0)} (${ratioText(r.soc_ratio)})`;
  $("tooltip").style.display="block"; $("tooltip").style.left=(e.clientX+14)+"px"; $("tooltip").style.top=(e.clientY+14)+"px";
}
function hideTip(){ $("tooltip").style.display="none"; }

function newOnly() {
  $("newOnly").innerHTML=DATA.new_only.map(r=>`<div class="case-mini"><b>${r.short}</b><div>mk ${fmt(r.new_mk,0)} · SOC ${fmt(r.new_soc,0)}</div><div>runtime ${r.runtime.toFixed(3)}s · deliverable ${fmt(r.deliverable_ms,0)}ms</div></div>`).join("");
}

function rows(records) {
  $("filterSummary").textContent=`显示 ${records.length}/36`;
  $("caseRows").innerHTML=records.map(r=>`<tr><td><b>${r.short}</b></td><td>${r.h}×${r.w}</td><td>${r.kind}</td><td>${status(r.base_ok)}</td><td>${status(r.new_ok)}</td><td>${fmt(r.base_mk,0)} → ${fmt(r.new_mk,0)}</td><td><span class="ratio-pill" style="color:${ratioColor(r.mk_ratio)};background:${ratioBg(r.mk_ratio)}">${ratioText(r.mk_ratio)}</span></td><td>${fmt(r.base_soc,0)} → ${fmt(r.new_soc,0)}</td><td><span class="ratio-pill" style="color:${ratioColor(r.soc_ratio)};background:${ratioBg(r.soc_ratio)}">${ratioText(r.soc_ratio)}</span></td><td>${r.runtime.toFixed(3)}s</td></tr>`).join("");
}

function render() {
  const rs=filtered(); qualityGroups(); scatter(rs); rows(rs);
}
cards(); coverage(); newOnly();
["metric","height","kind","sort"].forEach(id=>$(id).onchange=render);
$("search").oninput=render;
render();
</script>
</body>
</html>
"""


def main():
    data = build_data()
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    rendered = HTML.replace(
        "__DATA__", json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    )
    OUTPUT.write_text(rendered, encoding="utf-8")
    print("wrote", OUTPUT)


if __name__ == "__main__":
    main()
