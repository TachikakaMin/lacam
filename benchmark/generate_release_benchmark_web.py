#!/usr/bin/env python3
"""Generate the interactive 77-case release benchmark dashboard."""

import argparse
import csv
import hashlib
import html
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path


BENCH = Path(__file__).resolve().parent
REPO = BENCH.parent
DEFAULT_ROWS = BENCH / "results_release_77_20260904" / "rows.csv"
DEFAULT_TIMING = BENCH / "results_release_77_20260904" / "timing.json"
DEFAULT_OUT = BENCH / "viz_web" / "release_benchmark_77_20260904"
SUITE = BENCH / "release_benchmark.json"
VIZ_GENERATOR = BENCH / "generate_web_viz.py"


def _number(value, cast):
    if value in ("", None):
        return None
    return cast(value)


def _family_sort_key(name):
    if name == "warehouse_blocks":
        return (1, 0)
    if name.startswith("g") and "x" in name:
        try:
            return (0, int(name[1:].split("x", 1)[0]))
        except ValueError:
            pass
    return (0, 10**9, name)


def load_release_data(rows_path=DEFAULT_ROWS, timing_path=DEFAULT_TIMING):
    rows_path = Path(rows_path)
    timing_path = Path(timing_path)
    with rows_path.open(newline="") as stream:
        raw_rows = list(csv.DictReader(stream))
    timing = json.loads(timing_path.read_text())

    rows = []
    for raw in raw_rows:
        solved = raw["success"] == "1"
        rows.append(
            {
                "instance": raw["instance"],
                "family": raw["family"],
                "success": solved,
                "status": raw["status"],
                "makespan": _number(raw["executed_makespan"], int),
                "weighted_soc": _number(raw["weighted_soc"], float),
                "loaded_moves": _number(raw["loaded_moves"], int),
                "free_moves": _number(raw["free_moves"], int),
                "lift_drop": _number(raw["lift_drop"], int),
                "runtime_sec": _number(raw["runtime_sec"], float),
                "deliverable_ms": _number(raw["deliverable_ms"], float),
                "plan_sha256": raw["plan_sha256"],
                "raw": raw["raw"],
                "animation": (
                    f"cases/{raw['instance']}.html" if solved else None
                ),
            }
        )

    family_rows = []
    for family in sorted({row["family"] for row in rows},
                         key=_family_sort_key):
        selected = [row for row in rows if row["family"] == family]
        family_rows.append(
            {
                "family": family,
                "total": len(selected),
                "solved": sum(row["success"] for row in selected),
            }
        )

    warehouse = [
        row for row in rows if row["family"] == "warehouse_blocks"
    ]
    brap = [row for row in rows if row["family"] != "warehouse_blocks"]
    summary = {
        "total": len(rows),
        "solved": sum(row["success"] for row in rows),
        "warehouse_total": len(warehouse),
        "warehouse_solved": sum(row["success"] for row in warehouse),
        "brap_total": len(brap),
        "brap_solved": sum(row["success"] for row in brap),
        "animations": sum(row["animation"] is not None for row in rows),
        "wall_time_sec": float(timing["wall_time_sec"]),
        "jobs": int(timing["jobs"]),
        "timeout_sec": float(timing["timeout_per_run_sec"]),
    }
    return {
        "rows": rows,
        "families": family_rows,
        "warehouse": warehouse,
        "summary": summary,
        "timing": timing,
        "rows_path": str(rows_path),
        "timing_path": str(timing_path),
    }


def _fmt(value, digits=0):
    if value is None:
        return "—"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def render_dashboard(data):
    summary = data["summary"]
    timing = data["timing"]
    provenance = timing.get("provenance", {})
    suite = timing.get("suite", {})

    family_options = "\n".join(
        f'<option value="{html.escape(item["family"])}">'
        f'{html.escape(item["family"])}</option>'
        for item in data["families"]
    )
    family_bars = []
    for item in data["families"]:
        ratio = 100 * item["solved"] / item["total"]
        family_bars.append(
            f"""<div class="family-row">
  <div class="family-name">{html.escape(item['family'])}</div>
  <div class="bar"><span style="width:{ratio:.2f}%"></span></div>
  <div class="family-count">{item['solved']} / {item['total']}</div>
</div>"""
        )

    warehouse_cards = []
    for row in data["warehouse"]:
        warehouse_cards.append(
            f"""<article class="warehouse-card">
  <div class="case-state solved">✓ solved</div>
  <h3>{html.escape(row['instance'])}</h3>
  <div class="mini-metrics">
    <span>makespan <b>{_fmt(row['makespan'])}</b></span>
    <span>SOC <b>{_fmt(row['weighted_soc'], 0)}</b></span>
    <span>runtime <b>{_fmt(row['runtime_sec'], 3)} s</b></span>
  </div>
  <a class="play" href="{html.escape(row['animation'])}">▶ 播放动画</a>
</article>"""
        )

    table_rows = []
    for row in data["rows"]:
        status = "solved" if row["success"] else "timeout"
        status_label = "✓ solved" if row["success"] else "⏱ timeout"
        animation = (
            f'<a href="{html.escape(row["animation"])}">播放</a>'
            if row["animation"]
            else '<span class="muted">—</span>'
        )
        search_text = html.escape(
            f"{row['instance']} {row['family']}".lower(), quote=True
        )
        table_rows.append(
            f"""<tr class="case-row" data-family="{html.escape(row['family'], quote=True)}"
    data-status="{status}" data-search="{search_text}">
  <td class="instance">{html.escape(row['instance'])}</td>
  <td>{html.escape(row['family'])}</td>
  <td><span class="badge {status}">{status_label}</span></td>
  <td class="num">{_fmt(row['makespan'])}</td>
  <td class="num">{_fmt(row['weighted_soc'], 0)}</td>
  <td class="num">{_fmt(row['loaded_moves'])}</td>
  <td class="num">{_fmt(row['free_moves'])}</td>
  <td class="num">{_fmt(row['lift_drop'])}</td>
  <td class="num">{_fmt(row['runtime_sec'], 3)} s</td>
  <td>{animation}</td>
</tr>"""
        )

    template = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Carrier-LaCAM 正式 release benchmark · 77 cases</title>
<style>
:root {
  color-scheme: dark;
  --bg:#08101d; --panel:#111c2f; --panel2:#0c1627; --line:#263854;
  --text:#ecf4ff; --muted:#92a4bd; --blue:#4f9cff; --green:#36d399;
  --red:#ff7585; --amber:#f3bd55; --warehouse:#b98a52;
}
* { box-sizing:border-box; }
body {
  margin:0; color:var(--text);
  background:
    radial-gradient(circle at 12% 0%, #18366a 0, transparent 32rem),
    radial-gradient(circle at 92% 4%, #352248 0, transparent 30rem),
    var(--bg);
  font-family:-apple-system,BlinkMacSystemFont,"Segoe UI","Noto Sans SC",sans-serif;
}
main { max-width:1500px; margin:auto; padding:28px; }
h1 { margin:0; font-size:30px; letter-spacing:-.02em; }
h2 { font-size:20px; margin:0 0 14px; }
h3 { font-size:13px; line-height:1.45; overflow-wrap:anywhere; margin:8px 0 12px; }
a { color:#72b2ff; text-decoration:none; }
a:hover { text-decoration:underline; }
.lead { color:var(--muted); line-height:1.6; margin:8px 0 22px; }
.cards { display:grid; grid-template-columns:repeat(6,minmax(145px,1fr)); gap:12px; }
.card,.panel,.warehouse-card {
  border:1px solid var(--line); border-radius:14px;
  background:linear-gradient(145deg,rgba(18,31,52,.96),rgba(10,20,36,.96));
  box-shadow:0 18px 48px #0005;
}
.card { padding:16px; min-height:112px; }
.label { color:var(--muted); font-size:12px; letter-spacing:.06em; text-transform:uppercase; }
.value { font-size:29px; font-weight:780; margin:9px 0 4px; }
.detail { color:var(--muted); font-size:12px; line-height:1.45; }
.good { color:var(--green); }
.panel { padding:19px; margin-top:16px; }
.overview { display:grid; grid-template-columns:minmax(360px,.8fr) minmax(500px,1.2fr); gap:16px; }
.family-row { display:grid; grid-template-columns:125px 1fr 72px; gap:10px; align-items:center; margin:11px 0; }
.family-name { font-size:13px; color:#cad7e8; }
.bar { height:15px; border-radius:999px; background:#091322; border:1px solid #263753; overflow:hidden; }
.bar span { display:block; height:100%; background:linear-gradient(90deg,#347ce1,#47d1a1); border-radius:999px; }
.family-count { text-align:right; color:var(--muted); font-variant-numeric:tabular-nums; }
.protocol { color:var(--muted); line-height:1.65; font-size:13px; }
.protocol code { color:#d8e7fb; overflow-wrap:anywhere; }
.warehouse-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:12px; }
.warehouse-card { padding:14px; border-color:#5c4631; }
.case-state { font-size:12px; font-weight:700; }
.case-state.solved { color:var(--green); }
.mini-metrics { display:grid; grid-template-columns:repeat(3,1fr); gap:6px; margin-bottom:12px; }
.mini-metrics span { color:var(--muted); font-size:11px; background:#091322; padding:7px; border-radius:7px; }
.mini-metrics b { display:block; color:var(--text); margin-top:3px; font-size:13px; }
.play { display:inline-block; background:#1b5aaa; color:white; padding:7px 11px; border-radius:7px; font-size:12px; }
.controls {
  display:flex; flex-wrap:wrap; gap:10px; align-items:center;
  margin-bottom:13px; padding:11px; border:1px solid var(--line);
  border-radius:10px; background:#0a1425;
}
label { color:var(--muted); font-size:13px; }
select,input {
  margin-left:5px; background:#14223a; color:var(--text);
  border:1px solid #344a69; border-radius:7px; padding:7px 9px;
}
input { width:270px; }
#visibleCount { margin-left:auto; color:var(--muted); font-size:12px; }
.table-wrap { overflow:auto; max-height:900px; border:1px solid var(--line); border-radius:10px; }
table { width:100%; border-collapse:collapse; min-width:1160px; font-size:12px; }
th { position:sticky; top:0; z-index:2; background:#172640; color:#bed0e7; text-align:left; }
th,td { padding:9px 10px; border-bottom:1px solid #22344e; white-space:nowrap; }
tbody tr:hover { background:#14243c; }
.instance { font-family:ui-monospace,SFMono-Regular,Consolas,monospace; }
.num { text-align:right; font-variant-numeric:tabular-nums; }
.badge { display:inline-block; border-radius:999px; padding:3px 7px; font-weight:700; }
.badge.solved { color:#a8f0d2; background:#164c3b; }
.badge.timeout { color:#ffc4ca; background:#612d38; }
.muted { color:var(--muted); }
.foot { color:var(--muted); font-size:12px; line-height:1.6; margin-top:14px; }
@media(max-width:1100px) {
  .cards { grid-template-columns:repeat(3,1fr); }
  .overview { grid-template-columns:1fr; }
}
@media(max-width:760px) {
  main { padding:17px; }
  .cards,.warehouse-grid { grid-template-columns:1fr; }
  input { width:180px; }
}
</style>
</head>
<body>
<main>
  <h1>Carrier-LaCAM 正式 release benchmark</h1>
  <p class="lead">77 个真实 testcase 的同机 10 秒结果。成功实例可以直接播放经过权威 Python validator 重放确认的完整动画；失败实例保留 timeout 状态，不伪造计划。</p>

  <section class="cards">
    <article class="card"><div class="label">总体解出率</div><div class="value good">__SOLVED__ / __TOTAL__</div><div class="detail">全部正式 release cases</div></article>
    <article class="card"><div class="label">Warehouse-block</div><div class="value good">__WAREHOUSE_SOLVED__ / __WAREHOUSE_TOTAL__</div><div class="detail">全部可播放，包含 b3/d75</div></article>
    <article class="card"><div class="label">原 BRaP pool</div><div class="value">__BRAP_SOLVED__ / __BRAP_TOTAL__</div><div class="detail">与上一正式结果语义一致</div></article>
    <article class="card"><div class="label">动画</div><div class="value">__ANIMATIONS__</div><div class="detail">仅为合法成功计划生成</div></article>
    <article class="card"><div class="label">总 wall time</div><div class="value">__WALL__ s</div><div class="detail">14 个并行 worker</div></article>
    <article class="card"><div class="label">单例上限</div><div class="value">__TIMEOUT__ s</div><div class="detail">seed 0 · unit weights</div></article>
  </section>

  <section class="overview">
    <article class="panel">
      <h2>按 family 的成功率</h2>
      __FAMILY_BARS__
    </article>
    <article class="panel">
      <h2>协议与可审计信息</h2>
      <div class="protocol">
        Suite：<code>__SUITE__</code><br>
        Binary SHA-256：<code>__BINARY_SHA__</code><br>
        Binary source commit：<code>__COMMIT__</code><br>
        配置：carrier · seed 0 · unit weights · following allowed · jobs __JOBS__<br>
        <a href="summary.json">查看本页嵌入数据（JSON）</a>
      </div>
    </article>
  </section>

  <section class="panel">
    <h2>Warehouse-block：9 个实际 testcase</h2>
    <div class="warehouse-grid">__WAREHOUSE_CARDS__</div>
  </section>

  <section class="panel">
    <h2>全部 77 个 testcase</h2>
    <div class="controls">
      <label>Family<select id="familyFilter"><option value="all">全部</option>__FAMILY_OPTIONS__</select></label>
      <label>状态<select id="statusFilter"><option value="all">全部</option><option value="solved">solved</option><option value="timeout">timeout</option></select></label>
      <label>搜索<input id="searchBox" placeholder="实例名，例如 b3_a1_d75"></label>
      <span id="visibleCount"></span>
    </div>
    <div class="table-wrap">
      <table>
        <thead><tr><th>Instance</th><th>Family</th><th>Status</th><th>Makespan</th><th>Weighted SOC</th><th>Loaded</th><th>Free</th><th>Lift/Drop</th><th>Runtime</th><th>动画</th></tr></thead>
        <tbody>__TABLE_ROWS__</tbody>
      </table>
    </div>
    <p class="foot">runtime 是本轮并行 wall-clock 观测值；success、动作成本和 plan SHA 来自正式 rows.csv。动画生成前再次通过同一个权威 validator 校验。</p>
  </section>
</main>
<script>
const rows = [...document.querySelectorAll(".case-row")];
const family = document.getElementById("familyFilter");
const status = document.getElementById("statusFilter");
const search = document.getElementById("searchBox");
const count = document.getElementById("visibleCount");
function filterRows() {
  const needle = search.value.trim().toLowerCase();
  let visible = 0;
  for (const row of rows) {
    const show =
      (family.value === "all" || row.dataset.family === family.value) &&
      (status.value === "all" || row.dataset.status === status.value) &&
      (!needle || row.dataset.search.includes(needle));
    row.hidden = !show;
    if (show) visible++;
  }
  count.textContent = `显示 ${visible} / ${rows.length}`;
}
family.onchange = filterRows;
status.onchange = filterRows;
search.oninput = filterRows;
filterRows();
</script>
</body>
</html>
"""

    replacements = {
        "__SOLVED__": summary["solved"],
        "__TOTAL__": summary["total"],
        "__WAREHOUSE_SOLVED__": summary["warehouse_solved"],
        "__WAREHOUSE_TOTAL__": summary["warehouse_total"],
        "__BRAP_SOLVED__": summary["brap_solved"],
        "__BRAP_TOTAL__": summary["brap_total"],
        "__ANIMATIONS__": summary["animations"],
        "__WALL__": _fmt(summary["wall_time_sec"], 1),
        "__TIMEOUT__": _fmt(summary["timeout_sec"], 0),
        "__JOBS__": summary["jobs"],
        "__FAMILY_BARS__": "\n".join(family_bars),
        "__FAMILY_OPTIONS__": family_options,
        "__WAREHOUSE_CARDS__": "\n".join(warehouse_cards),
        "__TABLE_ROWS__": "\n".join(table_rows),
        "__SUITE__": html.escape(suite.get("name", "unknown")),
        "__BINARY_SHA__": html.escape(
            provenance.get("binary_sha256", "unknown")
        ),
        "__COMMIT__": html.escape(provenance.get("git_commit", "unknown")),
    }
    for key, value in replacements.items():
        template = template.replace(key, str(value))
    return template


def _instance_map():
    sys.path.insert(0, str(BENCH))
    from run_benchmark import discover_suite_cases

    _, cases, _ = discover_suite_cases(SUITE)
    return {path.stem: path for path, _ in cases}


def generate_case_visualizations(data, output_dir, jobs=8):
    output_dir = Path(output_dir)
    cases_dir = output_dir / "cases"
    cases_dir.mkdir(parents=True, exist_ok=True)
    instances = _instance_map()
    work = Path(data["rows_path"]).parent / "work"
    solved = [row for row in data["rows"] if row["success"]]

    def generate(row):
        name = row["instance"]
        instance = instances[name]
        plan = work / f"{name}.carrier.plan"
        if not plan.is_file():
            raise ValueError(f"missing plan for solved case: {plan}")
        digest = hashlib.sha256(plan.read_bytes()).hexdigest()
        if digest != row["plan_sha256"]:
            raise ValueError(
                f"plan SHA mismatch for {name}: {digest} != "
                f"{row['plan_sha256']}"
            )
        out = cases_dir / f"{name}.html"
        title = (
            f"{name} · makespan {row['makespan']} · "
            f"weighted SOC {_fmt(row['weighted_soc'], 0)}"
        )
        proc = subprocess.run(
            [
                sys.executable,
                str(VIZ_GENERATOR),
                str(instance),
                str(plan),
                str(out),
                "--title",
                title,
            ],
            capture_output=True,
            text=True,
            timeout=120,
        )
        if proc.returncode != 0:
            raise RuntimeError(
                f"visualization failed for {name}: "
                f"{(proc.stdout + proc.stderr)[-500:]}"
            )
        return name, out

    completed = []
    with ThreadPoolExecutor(max_workers=jobs) as executor:
        futures = {executor.submit(generate, row): row for row in solved}
        for future in as_completed(futures):
            name, out = future.result()
            completed.append(name)
            print(f"[{len(completed)}/{len(solved)}] {out.name}", flush=True)
    return sorted(completed)


def write_dashboard(data, output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "index.html").write_text(
        render_dashboard(data), encoding="utf-8"
    )
    payload = {
        "summary": data["summary"],
        "families": data["families"],
        "rows": data["rows"],
        "timing": data["timing"],
    }
    (output_dir / "summary.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rows", type=Path, default=DEFAULT_ROWS)
    parser.add_argument("--timing", type=Path, default=DEFAULT_TIMING)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--jobs", type=int, default=8)
    parser.add_argument("--skip-cases", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    data = load_release_data(args.rows, args.timing)
    if not args.skip_cases:
        generated = generate_case_visualizations(
            data, args.out_dir, jobs=args.jobs
        )
        if len(generated) != data["summary"]["animations"]:
            raise RuntimeError("not every solved case received an animation")
    write_dashboard(data, args.out_dir)
    print(f"dashboard={args.out_dir / 'index.html'}")
    print(
        f"cases={data['summary']['animations']} "
        f"solved={data['summary']['solved']}/{data['summary']['total']}"
    )


if __name__ == "__main__":
    main()
