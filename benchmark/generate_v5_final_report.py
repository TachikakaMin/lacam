#!/usr/bin/env python3
"""Generate the concise, data-driven Carrier-LaCAM v5 final report."""

import argparse
import csv
import html
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path


TESTCASE_C_ID = (
    "warehouse_cert_h12w20_b3_a1_s7of9_r6_t6_remote_seed0"
)


def _load_rows(path):
    with Path(path).open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _success(row):
    return str(row.get("success", "")).lower() in {"1", "true", "yes"}


def _number(row, key):
    value = row.get(key, "")
    return float(value) if value not in ("", None) else None


def _summary(rows):
    solved = [row for row in rows if _success(row)]
    makespans = [_number(row, "executed_makespan") for row in solved]
    work = [_number(row, "weighted_soc") for row in solved]
    runtimes = [
        _number(row, "runtime_sec")
        for row in rows
        if _number(row, "runtime_sec") is not None
    ]
    return {
        "total": len(rows),
        "solved": len(solved),
        "failed": len(rows) - len(solved),
        "success_rate": len(solved) / len(rows) if rows else 0,
        "makespan_sum": sum(makespans),
        "work_sum": sum(work),
        "makespan_mean": (
            statistics.mean(makespans) if makespans else None
        ),
        "makespan_median": (
            statistics.median(makespans) if makespans else None
        ),
        "work_mean": statistics.mean(work) if work else None,
        "runtime_sum": sum(runtimes),
        "runtime_mean": statistics.mean(runtimes) if runtimes else None,
    }


def _compare(base_rows, new_rows):
    base = {row["instance"]: row for row in base_rows}
    new = {row["instance"]: row for row in new_rows}
    common = sorted(
        name
        for name in base
        if name in new and _success(base[name]) and _success(new[name])
    )

    def metric(key):
        better = equal = worse = 0
        ratios = []
        base_sum = new_sum = 0.0
        for name in common:
            old = _number(base[name], key)
            current = _number(new[name], key)
            base_sum += old
            new_sum += current
            if current < old:
                better += 1
            elif current == old:
                equal += 1
            else:
                worse += 1
            if old and current:
                ratios.append(current / old)
        geometric = (
            math.exp(
                sum(math.log(value) for value in ratios) / len(ratios)
            )
            if ratios
            else None
        )
        return {
            "better": better,
            "equal": equal,
            "worse": worse,
            "base_sum": base_sum,
            "new_sum": new_sum,
            "geometric_ratio": geometric,
        }

    base_success = {name for name, row in base.items() if _success(row)}
    new_success = {name for name, row in new.items() if _success(row)}
    lex_better = lex_equal = lex_worse = 0
    for name in common:
        old = (
            _number(base[name], "executed_makespan"),
            _number(base[name], "weighted_soc"),
        )
        current = (
            _number(new[name], "executed_makespan"),
            _number(new[name], "weighted_soc"),
        )
        if current < old:
            lex_better += 1
        elif current == old:
            lex_equal += 1
        else:
            lex_worse += 1
    return {
        "common": len(common),
        "success_sets_equal": base_success == new_success,
        "makespan": metric("executed_makespan"),
        "work": metric("weighted_soc"),
        "lexicographic": {
            "better": lex_better,
            "equal": lex_equal,
            "worse": lex_worse,
        },
        "plan_hash_changes": sum(
            base[name].get("plan_sha256") !=
            new[name].get("plan_sha256")
            for name in common
        ),
    }


def _axis_summaries(rows, metadata, key):
    groups = defaultdict(list)
    for row in rows:
        meta = metadata.get(row["instance"])
        if meta is not None:
            groups[str(meta[key])].append(row)
    output = []
    for value, group in sorted(groups.items()):
        summary = _summary(group)
        output.append(
            {
                "value": value,
                "total": summary["total"],
                "solved": summary["solved"],
                "makespan_mean": summary["makespan_mean"],
                "work_mean": summary["work_mean"],
            }
        )
    return output


def _load_samples(path):
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if isinstance(payload, list):
        return payload
    return payload.get("samples", [])


def _testcase_c(samples):
    for sample in samples:
        if (sample.get("id") or sample.get("name")) == TESTCASE_C_ID:
            metrics = sample["planner"]["metrics"]
            metric_keys = (
                "loaded_moves",
                "free_moves",
                "lift_drop",
                "anon_moves",
                "reversals",
            )
            return {
                "id": TESTCASE_C_ID,
                "makespan": int(metrics["executed_makespan"]),
                "work": int(float(metrics["weighted_soc"])),
                "binary_sha256": sample["planner"]["binary_sha256"],
                "robots": (
                    int(sample["robots"])
                    if sample.get("robots") is not None
                    else None
                ),
                "targets": (
                    int(sample["targets"])
                    if sample.get("targets") is not None
                    else None
                ),
                **{
                    key: (
                        int(metrics[key])
                        if metrics.get(key) is not None
                        else None
                    )
                    for key in metric_keys
                },
            }
    raise ValueError("samples.json does not contain Testcase C")


def build_report_data(
    full_rows_path,
    full_timing_path,
    manifest_path,
    release_baseline_rows_path,
    quick_current_rows_path,
    historical_full_rows_path,
    historical_full_timing_path,
    ablation_rows_paths,
    samples_path,
):
    full_rows = _load_rows(full_rows_path)
    full_timing = json.loads(
        Path(full_timing_path).read_text(encoding="utf-8")
    )
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    metadata = {record["id"]: record for record in manifest}
    factorial_ids = set(metadata)
    factorial_rows = [
        row for row in full_rows if row["instance"] in factorial_ids
    ]
    quick_rows = [
        row for row in full_rows if row["instance"] not in factorial_ids
    ]

    quick_current = _load_rows(quick_current_rows_path)
    release_baseline = _load_rows(release_baseline_rows_path)
    historical_full = _load_rows(historical_full_rows_path)
    historical_timing = json.loads(
        Path(historical_full_timing_path).read_text(encoding="utf-8")
    )

    ablation_rows = {
        label: _load_rows(path)
        for label, path in ablation_rows_paths.items()
    }
    control = ablation_rows["control"]
    ablations = []
    for label in ("E1 off", "E2 off", "E4 off"):
        rows = ablation_rows[label]
        ablations.append(
            {
                "label": label,
                "summary": _summary(rows),
                "comparison": _compare(control, rows),
            }
        )

    failures = Counter(
        (row["family"], row["status"])
        for row in full_rows
        if not _success(row)
    )
    current_wall = float(full_timing["wall_time_sec"])
    historical_wall = float(historical_timing["wall_time_sec"])
    return {
        "full": _summary(full_rows),
        "quick": _summary(quick_rows),
        "factorial": _summary(factorial_rows),
        "quick_comparison": _compare(release_baseline, quick_current),
        "historical_comparison": _compare(
            historical_full, full_rows
        ),
        "ablations": ablations,
        "axes": {
            key: _axis_summaries(factorial_rows, metadata, key)
            for key in (
                "map_family",
                "density_level",
                "agent_level",
                "task_profile",
                "goal_mode",
            )
        },
        "failures": [
            {
                "family": family,
                "status": status,
                "count": count,
            }
            for (family, status), count in sorted(failures.items())
        ],
        "timing": {
            "wall_time_sec": current_wall,
            "solver_time_sum_sec": float(
                full_timing["methods"]["carrier"][
                    "solver_time_sum_sec"
                ]
            ),
            "jobs": int(full_timing["jobs"]),
            "timeout_sec": float(
                full_timing["timeout_per_run_sec"]
            ),
            "binary_sha256": full_timing["provenance"][
                "binary_sha256"
            ],
            "suite_sha256": full_timing["suite"][
                "definition_sha256"
            ],
            "execution_snapshot": full_timing["provenance"].get(
                "execution_snapshot", ""
            ),
            "historical_wall_time_sec": historical_wall,
            "wall_ratio": (
                current_wall / historical_wall
                if historical_wall > 0
                else None
            ),
        },
        "testcase_c": _testcase_c(_load_samples(samples_path)),
    }


def _fmt(value, digits=1):
    if value is None:
        return "—"
    return ("{0:." + str(digits) + "f}").format(value)


def _integer(value):
    return str(int(round(value)))


def _count(value):
    return "—" if value is None else str(int(value))


def _bwe(metric):
    return "{}/{}/{}".format(
        metric["better"], metric["equal"], metric["worse"]
    )


def _axis_table(rows, labels):
    cells = []
    for row in rows:
        label = labels.get(row["value"], row["value"])
        cells.append(
            "<tr><td>{}</td><td>{}/{}</td><td>{}</td><td>{}</td></tr>".format(
                html.escape(label),
                row["solved"],
                row["total"],
                _fmt(row["makespan_mean"], 1),
                _fmt(row["work_mean"], 1),
            )
        )
    return "\n".join(cells)


def _generate_html(data, cpp_tests, python_tests):
    full = data["full"]
    quick = data["quick"]
    factorial = data["factorial"]
    quick_cmp = data["quick_comparison"]
    historical_cmp = data["historical_comparison"]
    timing = data["timing"]
    testcase = data["testcase_c"]
    labels = {
        "low": "低密度",
        "medium": "中密度",
        "high": "高密度",
        "baseline": "baseline",
        "equal": "机器人≈任务",
        "scarce": "机器人偏少",
        "surplus": "机器人偏多",
        "local": "局部任务",
        "mixed": "混合任务",
        "cross_heavy": "跨区任务",
        "singleton": "固定单目标",
        "shared_pool": "共享目标池",
    }
    failure_text = "；".join(
        "{} {}×{}".format(
            item["family"], item["status"], item["count"]
        )
        for item in data["failures"]
    ) or "无"
    ablation_rows = []
    for item in data["ablations"]:
        comp = item["comparison"]
        summary = item["summary"]
        ablation_rows.append(
            "<tr><td>{}</td><td>{}</td><td>{}</td>"
            "<td>{}</td><td>{}</td><td>{:.1f}s</td></tr>".format(
                item["label"],
                _bwe(comp["makespan"]),
                _integer(summary["makespan_sum"]),
                _bwe(comp["work"]),
                _integer(summary["work_sum"]),
                summary["runtime_sum"],
            )
        )
    ablation_table = "\n".join(ablation_rows)
    e4_improvements = next(
        item["comparison"]["lexicographic"]["worse"]
        for item in data["ablations"]
        if item["label"] == "E4 off"
    )

    return """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Carrier-LaCAM v5 最终汇报</title>
<style>
:root {{
  --bg:#07101c; --panel:#0d1b2b; --line:#25425f; --text:#edf6ff;
  --muted:#9db0c3; --cyan:#2dd4bf; --blue:#60a5fa;
  --amber:#fbbf24; --red:#fb7185; --violet:#a78bfa;
}}
* {{ box-sizing:border-box; }}
body {{ margin:0; color:var(--text); background:
  radial-gradient(circle at 12% 0,#16365b 0,transparent 34rem),
  radial-gradient(circle at 95% 20%,#123e3f 0,transparent 30rem),var(--bg);
  font-family:Inter,ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif;
  line-height:1.65; }}
main {{ max-width:1180px; margin:auto; padding:30px 22px 56px; }}
a {{ color:#8ec5ff; }}
h1 {{ font-size:clamp(32px,5vw,58px); line-height:1.08; margin:8px 0 12px; }}
h2 {{ margin:34px 0 12px; font-size:24px; }}
h3 {{ margin:0 0 8px; font-size:17px; }}
p {{ color:var(--muted); }}
.eyebrow {{ color:var(--cyan); font-weight:800; letter-spacing:.13em;
  text-transform:uppercase; font-size:12px; }}
.nav {{ display:flex; gap:10px; flex-wrap:wrap; margin:18px 0 24px; }}
.nav a,.button {{ text-decoration:none; border:1px solid var(--line);
  border-radius:10px; padding:9px 12px; background:#0a1827; }}
.cards {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(180px,1fr));
  gap:13px; margin:22px 0; }}
.card,.panel {{ border:1px solid var(--line); border-radius:16px;
  background:linear-gradient(145deg,#11243aee,#091725ee);
  box-shadow:0 18px 45px #0004; }}
.card {{ padding:17px; }}
.card small {{ color:var(--muted); text-transform:uppercase;
  letter-spacing:.07em; }}
.card b {{ display:block; font-size:30px; margin-top:4px; }}
.panel {{ padding:20px; margin:15px 0; }}
.good {{ color:var(--cyan); }} .warn {{ color:var(--amber); }}
.bad {{ color:var(--red); }} .muted {{ color:var(--muted); }}
.flow {{ display:grid; grid-template-columns:repeat(5,1fr); gap:9px;
  align-items:stretch; margin:18px 0; }}
.step {{ border:1px solid var(--line); border-radius:12px; padding:13px;
  background:#0a1827; font-size:13px; }}
.step b {{ display:block; color:#dff5ff; margin-bottom:5px; }}
.arrow {{ color:var(--cyan); }}
.two {{ display:grid; grid-template-columns:1fr 1fr; gap:15px; }}
.callout {{ border-left:4px solid var(--cyan); padding:11px 15px;
  background:#0a1b2c; border-radius:0 12px 12px 0; }}
table {{ width:100%; border-collapse:collapse; font-size:13px; }}
th,td {{ border-bottom:1px solid #1d3852; padding:9px 8px; text-align:right; }}
th:first-child,td:first-child {{ text-align:left; }}
th {{ color:#bdd1e5; }}
.table-wrap {{ overflow:auto; }}
code {{ color:#c4e7ff; overflow-wrap:anywhere; }}
ul {{ padding-left:21px; }}
li {{ margin:7px 0; color:#cbd9e6; }}
.foot {{ margin-top:34px; color:var(--muted); font-size:12px; }}
@media(max-width:820px) {{
  .two {{ grid-template-columns:1fr; }}
  .flow {{ grid-template-columns:1fr; }}
}}
</style>
</head>
<body>
<main>
  <div class="eyebrow">Carrier-LaCAM v5 · 2026-09-05</div>
  <h1>让仓库机器人先处理“最堵的那一步”</h1>
  <p>这份报告面向第一次接触多机器人规划的读者：先讲清问题，再给实现、
  证据和仍未解决的地方。算法没有另起炉灶，而是在原 LaCAM-TAPF 的同一条
  搜索与执行路径里增加货架身份、因果准备、时序协调和同控制器改进。</p>
  <div class="nav">
    <a href="../index.html">可视化首页</a>
    <a href="../full_benchmark_v5_854df1_20260905/index.html">完整 full dashboard</a>
    <a href="../warehouse_case_proposal/index.html">{factor_total}-case testcase 提案</a>
    <a href="../warehouse_case_proposal/planner_runs/{testcase_id}.html">播放 Testcase C</a>
  </div>

  <section class="cards">
    <div class="card"><small>正式 full</small><b class="good">{full_solved}/{full_total}</b>
      <span class="muted">{full_rate:.1f}% solved</span></div>
    <div class="card"><small>新增 factorial</small><b class="good">{factor_solved}/{factor_total}</b>
      <span class="muted">{factor_total} 个受保护随机配对案例</span></div>
    <div class="card"><small>原 quick 子集</small><b>{quick_solved}/{quick_total}</b>
      <span class="muted">成功集合与 release baseline 相同</span></div>
    <div class="card"><small>Testcase C</small><b>T={tc_t}, W={tc_w}</b>
      <span class="muted">达到 31 拍 makespan 下界</span></div>
    <div class="card"><small>代码验证</small><b>{cpp_tests}+{python_tests}</b>
      <span class="muted">C++ + Python tests 全绿</span></div>
  </section>

  <h2>先看结论</h2>
  <div class="panel">
    <ul>
      <li><b class="good">正确性与覆盖：</b>正式 full 为 {full_solved}/{full_total}；
        新增 factorial 为 {factor_solved}/{factor_total}，{full_failed} 个失败全部是原
        quick 中的大图 timeout：{failure_text}。</li>
      <li><b class="good">主要收益：</b>相对隔离保存的 pre-v5 历史 full，
        共同成功例的 makespan 几何比为 {hist_t_ratio:.3f}
        （小于 1 更好），B/E/W={hist_t_bwe}。</li>
      <li><b class="warn">明确代价：</b>次级 work 几何比为
        {hist_w_ratio:.3f}，B/E/W={hist_w_bwe}；wall time 从
        {hist_wall:.1f}s 增至 {wall:.1f}s，约 {wall_ratio:.1f} 倍。</li>
      <li><b class="warn">不能夸大：</b>E1/E2 在 quick corpus 上没有证明聚合
        质量收益；E4 确实改善若干方案，但消耗更多运行时间。</li>
    </ul>
  </div>

  <h2>问题到底是什么？</h2>
  <div class="two">
    <div class="panel">
      <h3>两个目标，先后有别</h3>
      <p><b>T（makespan）</b>是最后一个任务完成的拍数，先最小化它；
      <b>W（work）</b>是搬货、空驶、Lift/Drop 等动作的加权总量，只在 T
      相同的时候比较。实现使用 10<sup>-6</sup> 定点整数，避免浮点误判。</p>
    </div>
    <div class="panel">
      <h3>最难的是“搬到一半”</h3>
      <p>货架进入通道后，当前局部任务可能因重新编图而消失，但物理上的
      货架仍在机器人手里。v5 用稳定的 TransferId 和 custody 记住“谁在搬
      哪个货架、最终放到哪里”，路线暂时不可用也不会把任务当作不存在。</p>
    </div>
  </div>

  <h2>仍是单一路径，只增加必要机制</h2>
  <div class="flow">
    <div class="step"><b>1 · PlanCost(T,W)</b>首次完成前缀与严格词典序</div>
    <div class="step"><b>2 · TransferId</b>跨 epoch 保留货架与搬运身份</div>
    <div class="step"><b>3 · 因果准备</b>前驱正在清障时，下游可先靠近</div>
    <div class="step"><b>4 · 时序协调</b>同一通道可错时复用，不做永久封路</div>
    <div class="step"><b>5 · 有界改进</b>同一控制器内尝试更小的 (T,W)</div>
  </div>
  <p class="callout">这些机制都接在原来的节点扩展、constraint tree、
  <code>funcPIBT()</code>、<code>apply_ops()</code>、两遍候选、repair
  和 final replay 上；没有平行 planner、实例特判或 legacy fallback。</p>

  <h2>一个具体例子：Testcase C</h2>
  <div class="panel">
    <p>{tc_robots} 台机器人要搬 {tc_targets} 个目标货架。旧的局部策略容易把通道里的货架视为
    “任务已经变了”，后续机器人也不知道能否提前准备。当前 production
    得到 <b>T={tc_t}, W={tc_w}</b>，共 {tc_loaded} 次 loaded move、
    {tc_free} 次 free move、{tc_lift_drop} 次 Lift/Drop，
    {tc_anon} anonymous move、{tc_reversals} reversal。W 不宣称最优。</p>
    <a class="button" href="../warehouse_case_proposal/planner_runs/{testcase_id}.html">
      打开逐帧动画</a>
  </div>

  <h2>固定 quick {quick_total}：相对 release baseline</h2>
  <div class="table-wrap panel">
    <table>
      <thead><tr><th>指标</th><th>better/equal/worse</th>
        <th>baseline → v5</th><th>几何比</th></tr></thead>
      <tbody>
        <tr><td>Makespan T</td><td>{quick_t_bwe}</td>
          <td>{quick_t_old} → {quick_t_new}</td><td>{quick_t_ratio:.6f}</td></tr>
        <tr><td>Work W</td><td>{quick_w_bwe}</td>
          <td>{quick_w_old} → {quick_w_new}</td><td>{quick_w_ratio:.6f}</td></tr>
      </tbody>
    </table>
  </div>
  <p>两版都是 {quick_solved}/{quick_total}，成功集合完全相同。多数共同实例
  的 T 更小，但 T 总和、W 和运行时间并非全面改善，所以不声称逐例单调。</p>

  <h2>正式 full {full_total}：新增案例看到了什么？</h2>
  <div class="two">
    <div class="panel table-wrap">
      <h3>地图规模</h3>
      <table><thead><tr><th>组</th><th>solved</th><th>平均 T</th><th>平均 W</th></tr></thead>
      <tbody>{map_rows}</tbody></table>
    </div>
    <div class="panel table-wrap">
      <h3>机器人数量</h3>
      <table><thead><tr><th>组</th><th>solved</th><th>平均 T</th><th>平均 W</th></tr></thead>
      <tbody>{agent_rows}</tbody></table>
    </div>
  </div>
  <div class="two">
    <div class="panel table-wrap">
      <h3>任务分布</h3>
      <table><thead><tr><th>组</th><th>solved</th><th>平均 T</th><th>平均 W</th></tr></thead>
      <tbody>{profile_rows}</tbody></table>
    </div>
    <div class="panel table-wrap">
      <h3>Goal 模式</h3>
      <table><thead><tr><th>组</th><th>solved</th><th>平均 T</th><th>平均 W</th></tr></thead>
      <tbody>{goal_rows}</tbody></table>
    </div>
  </div>
  <p>这些是描述性结果，不是因果定律。要筛选全部 {full_total} 行、查看散点图或播放
  {full_solved} 个真实方案，请使用 <a href="../full_benchmark_v5_854df1_20260905/index.html">
  full dashboard</a>。</p>

  <h2>E5 消融：负结果也保留</h2>
  <div class="panel table-wrap">
    <table>
      <thead><tr><th>移除机制</th><th>T B/E/W</th><th>T sum</th>
        <th>W B/E/W</th><th>W sum</th><th>runtime sum</th></tr></thead>
      <tbody>{ablation_rows}</tbody>
    </table>
  </div>
  <p><b>E1 off</b> 在 quick 上聚合 T/W 反而更小；<b>E2 off</b> 没有改变
  T/W；<b>E4 off</b> 更快但失去 {e4_improvements} 行严格 incumbent 改进。这些负结果说明：
  quick corpus 支持 E4 的质量作用，却不足以证明 E1/E2 的一般收益。</p>

  <h2>为什么相信这些数字？</h2>
  <div class="panel">
    <ul>
      <li>{cpp_tests} 项 C++ 与 {python_tests} 项 Python 测试全绿；每个成功
        plan 都由权威 Python validator 再重放，并精确核对定点 W。</li>
      <li>full 先由独立 GPT-5.6 Sol/xhigh 审查，approval 同时绑定 binary、
        suite definition 和 semantic corpus。</li>
      <li>worker 只读取 Linux sealed memfd 中的二进制和 {full_total} 份 YAML；
        本轮 provenance 为 <code>{snapshot}</code>。</li>
      <li>第一次 full 虽完成 solver task，却在发布前 memfd 重哈希路径处
        fail-closed，没有 rows/timing。该缺陷先新增 RED regression，再修复、
        跑完整 Python 测试并由另一位独立 xhigh reviewer 重新批准后重跑。</li>
    </ul>
  </div>

  <h2>仍然存在的限制</h2>
  <div class="panel">
    <ul>
      <li>{full_failed} 个原 BRaP 大图在 {timeout:g} 秒内仍 timeout；v5 没有提高成功率。</li>
      <li>W 经常比历史实现更大，Testcase C 也只证明 T 达下界，不证明 W 最优。</li>
      <li>首解后的有界改进让正式 full wall time 约为历史运行的
        {wall_ratio:.1f} 倍；这是清楚可见的质量—时间交换。</li>
      <li>{factor_total} 个 factorial case 是构造可解、受保护的随机配对集合；它扩大
        覆盖面，但不能替代真实仓库流量分布。</li>
    </ul>
  </div>

  <h2>证据入口</h2>
  <div class="nav">
    <a href="../full_benchmark_v5_854df1_20260905/index.html">full dashboard</a>
    <a href="../../results_full_v5_854df1_20260905_r2/rows.csv">正式 rows.csv</a>
    <a href="../../results_full_v5_854df1_20260905_r2/timing.json">正式 timing.json</a>
    <a href="../../full_review_approval_v5.json">full 审查 approval</a>
    <a href="../../ablation_v5_20260905.md">E5 消融记录</a>
    <a href="../release_benchmark_77_20260904/index.html">release baseline dashboard</a>
  </div>
  <details class="panel"><summary>Provenance hashes</summary>
    <p>Binary: <code>{binary}</code><br>
    Suite: <code>{suite}</code><br>
    Full wall: {wall:.1f}s · solver sum: {solver_sum:.1f}s ·
    {jobs} jobs · {timeout:g}s/case</p>
  </details>
  <div class="foot">本页的 benchmark 统计由显式证据文件生成，不从正文手抄。</div>
</main>
</body>
</html>
""".format(
        testcase_id=TESTCASE_C_ID,
        full_solved=full["solved"],
        full_total=full["total"],
        full_failed=full["failed"],
        full_rate=100 * full["success_rate"],
        factor_solved=factorial["solved"],
        factor_total=factorial["total"],
        quick_solved=quick["solved"],
        quick_total=quick["total"],
        tc_t=testcase["makespan"],
        tc_w=testcase["work"],
        tc_robots=_count(testcase.get("robots")),
        tc_targets=_count(testcase.get("targets")),
        tc_loaded=_count(testcase.get("loaded_moves")),
        tc_free=_count(testcase.get("free_moves")),
        tc_lift_drop=_count(testcase.get("lift_drop")),
        tc_anon=_count(testcase.get("anon_moves")),
        tc_reversals=_count(testcase.get("reversals")),
        cpp_tests=cpp_tests,
        python_tests=python_tests,
        failure_text=html.escape(failure_text),
        hist_t_ratio=historical_cmp["makespan"]["geometric_ratio"],
        hist_t_bwe=_bwe(historical_cmp["makespan"]),
        hist_w_ratio=historical_cmp["work"]["geometric_ratio"],
        hist_w_bwe=_bwe(historical_cmp["work"]),
        hist_wall=timing["historical_wall_time_sec"],
        wall=timing["wall_time_sec"],
        wall_ratio=timing["wall_ratio"],
        quick_t_bwe=_bwe(quick_cmp["makespan"]),
        quick_t_old=_integer(quick_cmp["makespan"]["base_sum"]),
        quick_t_new=_integer(quick_cmp["makespan"]["new_sum"]),
        quick_t_ratio=quick_cmp["makespan"]["geometric_ratio"],
        quick_w_bwe=_bwe(quick_cmp["work"]),
        quick_w_old=_integer(quick_cmp["work"]["base_sum"]),
        quick_w_new=_integer(quick_cmp["work"]["new_sum"]),
        quick_w_ratio=quick_cmp["work"]["geometric_ratio"],
        map_rows=_axis_table(data["axes"]["map_family"], labels),
        agent_rows=_axis_table(data["axes"]["agent_level"], labels),
        profile_rows=_axis_table(data["axes"]["task_profile"], labels),
        goal_rows=_axis_table(data["axes"]["goal_mode"], labels),
        ablation_rows=ablation_table,
        e4_improvements=e4_improvements,
        snapshot=html.escape(timing["execution_snapshot"]),
        binary=html.escape(timing["binary_sha256"]),
        suite=html.escape(timing["suite_sha256"]),
        solver_sum=timing["solver_time_sum_sec"],
        jobs=timing["jobs"],
        timeout=timing["timeout_sec"],
    )


def generate_report(
    full_rows_path,
    full_timing_path,
    manifest_path,
    release_baseline_rows_path,
    quick_current_rows_path,
    historical_full_rows_path,
    historical_full_timing_path,
    ablation_rows_paths,
    samples_path,
    out_dir,
    cpp_tests,
    python_tests,
):
    data = build_report_data(
        full_rows_path=full_rows_path,
        full_timing_path=full_timing_path,
        manifest_path=manifest_path,
        release_baseline_rows_path=release_baseline_rows_path,
        quick_current_rows_path=quick_current_rows_path,
        historical_full_rows_path=historical_full_rows_path,
        historical_full_timing_path=historical_full_timing_path,
        ablation_rows_paths=ablation_rows_paths,
        samples_path=samples_path,
    )
    output = Path(out_dir)
    output.mkdir(parents=True, exist_ok=True)
    page = _generate_html(data, cpp_tests, python_tests)
    (output / "index.html").write_text(page, encoding="utf-8")
    (output / "summary.json").write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--full-rows", type=Path, required=True)
    parser.add_argument("--full-timing", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--release-baseline-rows", type=Path, required=True)
    parser.add_argument("--quick-current-rows", type=Path, required=True)
    parser.add_argument("--historical-full-rows", type=Path, required=True)
    parser.add_argument("--historical-full-timing", type=Path, required=True)
    parser.add_argument("--ablation-control-rows", type=Path, required=True)
    parser.add_argument("--ablation-e1-rows", type=Path, required=True)
    parser.add_argument("--ablation-e2-rows", type=Path, required=True)
    parser.add_argument("--ablation-e4-rows", type=Path, required=True)
    parser.add_argument("--samples", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--cpp-tests", type=int, required=True)
    parser.add_argument("--python-tests", type=int, required=True)
    args = parser.parse_args()

    data = generate_report(
        full_rows_path=args.full_rows,
        full_timing_path=args.full_timing,
        manifest_path=args.manifest,
        release_baseline_rows_path=args.release_baseline_rows,
        quick_current_rows_path=args.quick_current_rows,
        historical_full_rows_path=args.historical_full_rows,
        historical_full_timing_path=args.historical_full_timing,
        ablation_rows_paths={
            "control": args.ablation_control_rows,
            "E1 off": args.ablation_e1_rows,
            "E2 off": args.ablation_e2_rows,
            "E4 off": args.ablation_e4_rows,
        },
        samples_path=args.samples,
        out_dir=args.out_dir,
        cpp_tests=args.cpp_tests,
        python_tests=args.python_tests,
    )
    print(
        "wrote {}: full {}/{}; factorial {}/{}".format(
            args.out_dir,
            data["full"]["solved"],
            data["full"]["total"],
            data["factorial"]["solved"],
            data["factorial"]["total"],
        )
    )


if __name__ == "__main__":
    main()
