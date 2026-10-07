"""Build a self-contained comparison page around audited paper-style figures."""
import argparse
import csv
import html
import json
from pathlib import Path


def group_extension_section(run):
    source = run.parent / 'group_250_500_cache_comparison'
    if not (source / 'completion_audit.json').exists():
        return ''
    completion = json.loads((source / 'completion_audit.json').read_text())
    assert completion['complete']
    assert json.loads((source / 'coverage_audit.json').read_text())['complete']
    records = json.loads((source / 'before_after_summary.json').read_text())
    labels = {'ITA-LaCAM': 'ITA-LaCAM', 'IR-TAPF (original cache)': 'IR 原版缓存',
              'IR-TAPF (cache fixed)': 'IR 修复缓存'}
    ordered = ['ITA-LaCAM', 'IR-TAPF (original cache)', 'IR-TAPF (cache fixed)']
    rows = ''.join('<tr><td>' + labels[name] + '</td><td>' + str(next(r for r in records if r['version'] == name)['solved']) + ' / 960</td><td><strong>' + format(next(r for r in records if r['version'] == name)['success_rate'], '.2f') + '%</strong></td></tr>' for name in ordered)
    base = 'group_extension/cache_fixed/'
    return '<section id="group-extension"><span class="tag">新增 Group 扩展 · 全量核查通过</span><h2>Group 扩展：250、300、350、400、450、500 agents</h2><p>8 张地图，每个规模 20 个实例，共 960 个新实例。只跑 ITA-LaCAM 和 IR-TAPF；搜索预算均为 10 秒，56 路求解并行，预留 4 个逻辑 CPU。IR 原版结果保留，缓存修复版使用完全相同的输入另跑一次。</p><div class="scroll"><table><thead><tr><th>新扩展方法 / 版本</th><th>成功 / 实例</th><th>成功率</th></tr></thead><tbody>' + rows + '</tbody></table></div><h3>原 Group 图延伸到 500 agents</h3><a href="' + base + 'group_10_500_success.png"><img style="width:100%;height:auto" src="' + base + 'group_10_500_success.png" alt="Group 10 到 500 agents 完整曲线，区分原版和修复版 IR"></a><p class="small">浅蓝背景是新实验区间。10–200 使用原有统计；IR 缓存修复版只测了 250–500，蓝线仅显示实测区间。250–500 没有论文对应数值。旧统计含已授权的超时重放，新统计各版本各测例只运行一次。</p><details><summary>放大查看 250–500 agents 的三条曲线</summary><img style="width:100%" src="' + base + 'group_success_before_after.png" alt="250 到 500 agents 修复前后对比"></details><h3>同一批双方成功测例：SOC 与首解时间</h3><p class="small">下面使用修复版 IR，与同测例 LaCAM 配对。按地图与规模取中位数；未解出的测例不填成 0。</p><div class="panels"><article class="panel"><h3>SOC</h3><a href="' + base + 'group_soc.png"><img src="' + base + 'group_soc.png" alt="配对成功测例 SOC"></a></article><article class="panel"><h3>首解时间</h3><a href="' + base + 'group_first_ms.png"><img src="' + base + 'group_first_ms.png" alt="配对成功测例首解时间"></a></article></div><div class="note"><p><strong>为什么有 IR 成功、LaCAM 失败？</strong>已核查 25 个反例。冻结 C++ LaCAM 自己的首次目标分配，15 个能成功；仅恢复已知节点重访置顶，17 个能成功。诊断轨迹和控制结果单独保留，未改写主实验成功率。</p><a href="' + base + 'lacam_failure_analysis/analysis.html">打开 25 个反例的控制实验与真实搜索回放</a></div><div class="downloads"><a href="' + base + 'group_10_500_success.pdf">10–500 图 PDF</a><a href="' + base + 'group_10_500_success_rates.csv">10–500 成功率 CSV</a><a href="' + base + 'ir_before_after_cases.csv">IR 修复前后逐测例 CSV</a><a href="' + base + 'paired_comparisons.csv">配对 SOC / 时间 CSV</a><a href="' + base + 'coverage_audit.json">新结果轨迹复核</a><a href="' + base + 'comparison.html">扩展实验完整详情</a></div></section>'


def build(run):
    audit = json.loads((run / 'coverage_audit.json').read_text())
    assert audit['complete'], 'Publish comparisons only after the archive audit passes'
    summary = json.loads((run / 'summary.json').read_text())
    baseline = json.loads((run / 'before_timeout_review' / 'summary.json').read_text())
    review = json.loads((run / 'timeout_review_summary.json').read_text())
    with (run / 'paper_comparison' / 'paper_vs_rerun_metrics.csv').open() as stream:
        metrics = list(csv.DictReader(stream))
    labels = {'lacam_dfs': 'ITA-LaCAM', 'ir': 'IR-TAPF', 'itacbs': 'ITA-CBS'}
    paper = {'group': {'lacam_dfs': 100, 'ir': 94.9, 'itacbs': 15.9},
             'common': {'lacam_dfs': 100, 'ir': 96, 'itacbs': 67.8}}
    rows = []
    for row in summary:
        old = next(x for x in baseline if x['suite'] == row['suite'] and x['method'] == row['method'])
        reported = paper[row['suite']][row['method']]
        measured = 100 * row['success_rate']
        rows.append(f'<tr><td>{row["suite"].title()}</td><td>{labels[row["method"]]}</td>'
                    f'<td>{reported:.2f}%</td><td>{100 * old["success_rate"]:.2f}%</td>'
                    f'<td><strong>{measured:.2f}%</strong></td><td>{measured - reported:+.2f} pp</td>'
                    f'<td>{row["solved"]:,} / {row["completed"]:,}</td></tr>')
    data = json.dumps({'metrics': metrics, 'review': {k: v for k, v in review.items() if k != 'records'},
                       'audit': audit}, ensure_ascii=False).replace('</', '<\\/')
    template = r'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>TAPF · 论文与复核结果对比</title>
<style>:root{--ink:#17283b;--sub:#5c6d7e;--blue:#1263ba;--edge:#dae3eb}*{box-sizing:border-box}body{margin:0;background:#f4f7fa;color:var(--ink);font:16px system-ui}main{max-width:1450px;margin:auto;padding:36px 24px}h1{font-size:34px;margin:0 0 12px}h2{font-size:23px}p{line-height:1.75;color:var(--sub)}a{color:var(--blue)}.cards{display:grid;grid-template-columns:repeat(4,1fr);gap:14px}.card,section{border:1px solid var(--edge);background:white;border-radius:14px;padding:24px;margin:20px 0}.card{margin:0}.card strong{display:block;font-size:29px;margin-top:8px}.tag{display:inline-block;border-radius:20px;background:#e8f5ee;color:#0b7145;padding:7px 14px;font-weight:700}.scroll{overflow-x:auto}table{border-collapse:collapse;width:100%;white-space:nowrap}td,th{padding:13px;border-bottom:1px solid var(--edge);text-align:left;font-size:14px}th{color:var(--sub)}.tabs{display:flex;gap:8px;flex-wrap:wrap}.tabs button,button{padding:10px 16px;border:1px solid var(--edge);border-radius:8px;background:white;color:var(--ink);cursor:pointer}.tabs button.active{background:var(--blue);color:white;border-color:var(--blue)}.panels{display:grid;grid-template-columns:1fr 1fr;gap:18px;margin-top:20px}.panel{min-width:0;background:#fff;border:1px solid var(--edge);border-radius:10px;padding:14px}.panel h3{font-size:17px;margin:0 0 12px}.panel img{width:100%;height:auto;display:block}.only-rerun .paper{display:none}.only-rerun .panels{grid-template-columns:1fr}.note{background:#edf4fc;border-left:4px solid var(--blue);padding:14px 20px}.downloads{display:flex;gap:18px;flex-wrap:wrap;margin-top:18px}pre{white-space:pre-wrap;overflow-wrap:anywhere;font:13px ui-monospace}details{margin:15px 0}summary{cursor:pointer}.small{font-size:13px}@media(max-width:800px){main{padding:22px 12px}.cards{grid-template-columns:repeat(2,1fr)}.panels{grid-template-columns:1fr}h1{font-size:26px}section{padding:17px}}</style></head>
<body><main><span class="tag">全量轨迹复核通过</span><h1>论文与复核结果，逐图对比</h1><p>使用仓库原生成脚本。8 张地图，9,760 个实例，3 种方法，每次搜索 10 秒。左侧直接显示论文 PDF 原图，右侧使用实际运行数据重画。</p>
<div class="cards"><div class="card">完整运行记录<strong>29,280</strong></div><div class="card">成功轨迹复核<strong>__CHECKED__</strong></div><div class="card">Common LaCAM<strong>100.00%</strong></div><div class="card">当前基础设施异常<strong>0</strong></div></div>
<section><h2>成功率：论文、原批次与复核后</h2><div class="scroll"><table><thead><tr><th>实验</th><th>方法</th><th>论文</th><th>原批次</th><th>复核后</th><th>与论文差值</th><th>有效解 / 实例</th></tr></thead><tbody>__ROWS__</tbody></table></div><p class="small">pp 表示百分点。复核后统计加入了用户授权的重放结果，原批次数据完整保留。不同硬件和重新生成的实例会影响数值。</p></section>
<section id="figures"><h2>与论文一致的图形定义</h2><div class="tabs" id="tabs"></div><p id="description"></p><label><input type="checkbox" id="only" onchange="document.getElementById('figures').classList.toggle('only-rerun',this.checked)"> 仅显示重跑图</label>
<div class="panels"><article class="panel paper"><h3>论文原图 · PDF 裁切</h3><a id="paperLink" target="_blank"><img id="paperImage" alt="论文原图"></a></article><article class="panel"><h3>复核后结果 · 实际数据</h3><a id="rerunLink" target="_blank"><img id="rerunImage" alt="复核后的实际结果"></a></article></div><p class="small">点击图片打开原尺寸。论文图是原始 PDF 图片，未通过图像估计或拟造原始数据点。</p><div class="downloads"><a id="pdfLink">下载重跑 PDF</a><a id="comparisonLink">下载并排 PNG</a><a href="paper_comparison/paper_vs_rerun_metrics.csv">指标对比 CSV</a></div></section>
<section><h2>复核做了什么</h2><div class="note"><p>Common LaCAM 的三个超时实例，以同样的输入、seed 和 10 秒预算重放，轨迹验证通过后纳入统计。其余 5,404 个未求解记录全部重新尝试一次，采用 56 个不同物理核心运行求解任务。</p><p id="reviewText"></p></div><p>每份成功归档轨迹都通过独立进程检查：起点、目标资格、地图边界、障碍、移动距离、顶点冲突、交换冲突及重新计算的 SOC。全部输入哈希、二进制哈希、实例覆盖和预留 CPU 也已检查。</p><div class="downloads"><a href="timeout_review_summary.json">5,404 条复核明细</a><a href="before_timeout_review/rows.csv">原批次 CSV</a><a href="rows.csv">当前 CSV</a><a href="coverage_audit.json">最终复核证明</a><a href="common_unsolved_diagnosis/index.html">三个 Common 超时案例</a></div></section>
<section><h2>生成方式与比较边界</h2><p>Common 使用仓库原脚本内部的 seed=1 连续抽样。Group 保留原脚本的采样逻辑，外部固定初始 seed=1；旧 Group 文件的历史随机数状态已丢失。所有方法读取同一份地图几何，Symbotic 的可通行标记已统一为点号。</p><p>机器为 Threadripper 3990X，论文使用 i9-12900K。首解时间使用同一 10 秒墙钟预算。图 5、6 的中位数曲线采用 20 个等宽对数 SOC 分箱；阴影表示本页分箱内 10%–90% 分位；论文没有给出精确分箱边界或阴影分位定义。图中的范围外点数量已标注，CSV 保留全部数据。</p><div class="downloads"><a href="https://arxiv.org/abs/2609.14208" target="_blank">论文 arXiv</a><a href="paper.pdf">论文 PDF</a><a href="generation_policy.json">生成策略</a><a href="native_generator_audit.json">原脚本检查</a><a href="dataset_audit.json">数据检查</a><a href="index.html">运行监控与真实日志</a></div><details><summary>展开完整指标与验证数据</summary><pre id="raw"></pre></details></section>
</main><script>const DATA=__DATA__;const figures={2:['图 2 · Group 成功率','8 张地图、3 种方法；横轴为 agents 数，纵轴为成功率。'],3:['图 3 · Common 成功率','保持论文 2×4 排列；颜色表示方法，线型表示 0%、30%、60%、100% 共享比例。'],4:['图 4 · 首解时间','横轴 IR-TAPF、纵轴 ITA-LaCAM；双对数坐标，范围 1 ms–10 s。未求解点记为 10 s。'],5:['图 5 · 相对 IR 的 SOC','横轴 IR-TAPF SOC，纵轴 IR-TAPF SOC / ITA-LaCAM SOC。比值大于 1 表示 LaCAM 成本更低。'],6:['图 6 · 相对 CBS 的 SOC','横轴 ITA-CBS SOC，纵轴 ITA-LaCAM SOC / ITA-CBS SOC。比值 1 表示达到 CBS 的最优成本。']};function show(n){document.querySelectorAll('#tabs button').forEach(b=>b.classList.toggle('active',Number(b.dataset.figure)===n));document.getElementById('description').textContent=figures[n][1];for(const kind of ['paper','rerun']){const url='paper_comparison/'+kind+'_figure'+n+'.png';document.getElementById(kind+'Image').src=url;document.getElementById(kind+'Link').href=url}document.getElementById('pdfLink').href='paper_comparison/rerun_figure'+n+'.pdf';document.getElementById('comparisonLink').href='paper_comparison/comparison_figure'+n+'.png'}document.getElementById('tabs').innerHTML=Object.entries(figures).map(([n,v])=>'<button data-figure="'+n+'" onclick="show('+n+')">'+v[0]+'</button>').join('');const totals=DATA.review.by_method;document.getElementById('reviewText').textContent='CBS 复核 '+totals.itacbs.reviewed+' 条，新增成功 '+totals.itacbs.newly_solved+' 条，仍未求解 '+totals.itacbs.still_unsolved+' 条；IR 复核 '+totals.ir.reviewed+' 条，新增成功 '+totals.ir.newly_solved+' 条，仍未求解 '+totals.ir.still_unsolved+' 条。';document.getElementById('raw').textContent=JSON.stringify(DATA,null,2);show(2);</script></body></html>'''
    page = template.replace('__CHECKED__', f'{audit["archived_solutions_checked"]:,}').replace('__ROWS__', ''.join(rows)).replace('__DATA__', data)
    extension = group_extension_section(run)
    full_fixed = run.parent / 'paper_2609_14208_cache_comparison'
    if (full_fixed / 'completion_audit.json').exists() and json.loads((full_fixed / 'completion_audit.json').read_text())['complete']:
        fresh = json.loads((full_fixed / 'summary.json').read_text())
        fresh_audit = json.loads((full_fixed / 'coverage_audit.json').read_text())
        assert fresh_audit['complete']
        page = page.replace('成功轨迹复核<strong>' + f'{audit["archived_solutions_checked"]:,}', '修复版对比轨迹复核<strong>' + f'{fresh_audit["archived_solutions_checked"]:,}')
        cells = ''.join('<tr><td>' + r['suite'].title() + '</td><td>' + str(r['solved']) + ' / ' + str(r['completed']) + '</td><td>' + format(100*r['success_rate'], '.2f') + '%</td></tr>' for r in fresh if r['method'] == 'ir')
        full_section = '<section id="full-ir-cache-fixed"><h2>已完成：IR 缓存修复版全量重跑</h2><p>9,760 个实例全部重新运行，未与原成功结果取最佳值。成功归档轨迹已独立复核。求解并行度由 56 提高到 112；每例仍为 10 秒墙钟搜索预算。并行度会影响单例速度，修复前后差异不能全部归因于缓存修复。原版统计和原图保留在下方。</p><table><tr><th>实验</th><th>修复版 IR 成功 / 总数</th><th>成功率</th></tr>' + cells + '</table><a href="full_ir_cache_fixed/comparison.html">查看修复版与论文逐图对比、10–500 完整曲线及逐例 CSV</a><img style="width:100%" src="full_ir_cache_fixed/group_10_500_success.png"></section>'
        controls = '<h3>修复版与论文：逐图对比</h3><select aria-label="选择论文图" onchange="document.getElementById(\'fixed-paper-image\').src=\'full_ir_cache_fixed/paper_comparison/comparison_figure\'+this.value+\'.png\'"><option value="2">图 2 · Group 成功率</option><option value="3">图 3 · Common 成功率</option><option value="4">图 4 · 首解时间</option><option value="5">图 5 · 相对 IR 的 SOC</option><option value="6">图 6 · 相对 CBS 的 SOC</option></select><img id="fixed-paper-image" style="width:100%" src="full_ir_cache_fixed/paper_comparison/comparison_figure2.png" alt="缓存修复版结果与论文原图对比">'
        full_section = full_section.replace('</section>', controls + '</section>')
        if (full_fixed / 'regression_control_summary.json').exists():
            diagnostic = json.loads((full_fixed / 'regression_control_summary.json').read_text())
            assert diagnostic['complete'] and diagnostic['solved'] == 51
            explanation = '<div class="note"><p><strong>为什么 Common 成功率下降？</strong>52 个原成功案例在本次 112 路阶段失败。相同修复版、相同输入与 10 秒预算，改为 8 路并行后，51 个 orz900d 案例全部成功，1 个 maze 案例仍失败。这表明结果对并行负载敏感。诊断成功未加入主统计。</p><a href="full_ir_cache_fixed/regression_controls.html">核对 52 个案例的 112 路与 8 路结果</a></div>'
            full_section = full_section.replace('<h3>修复版与论文：逐图对比</h3>', explanation + '<h3>修复版与论文：逐图对比</h3>')
        page = page.replace('<div class="cards">', full_section + '<div class="cards">', 1)
    elif (run.parent / 'paper_2609_14208_cache_fixed' / 'status.json').exists():
        progress = '<section id="full-ir-cache-fixed"><h2>进行中：IR 缓存修复版全量重跑</h2><p>全部 9,760 个实例，112 路求解与 8 路验证，预留 4 个逻辑 CPU。完成验证后将公布完整统计和论文对比图。</p><p id="fixed-progress">正在读取进度…</p></section><script>async function refreshFixed(){try{const s=await fetch("full_ir_cache_fixed_runs/status.json?t="+Date.now()).then(r=>r.json());let text="已完成 "+s.completed+" / "+s.total+"；状态："+s.state+"；基础设施异常："+(s.infrastructure_errors||0);if(s.state==="solvers_finished"){text+="；正在核查归档轨迹和生成图表";try{const a=await fetch("full_ir_cache_fixed/status.json?t="+Date.now()).then(r=>r.json());if(a.archive_validation_total)text+="；归档复核 "+a.archive_validation_completed+" / "+a.archive_validation_total}catch(e){}}document.getElementById("fixed-progress").textContent=text}catch(e){}}refreshFixed();setInterval(refreshFixed,10000);</script>'
        page = page.replace('<div class="cards">', progress + '<div class="cards">', 1)
    if (run / 'cache_bug_review' / 'archive_audit.json').exists():
        cache_audit = json.loads((run / 'cache_bug_review' / 'archive_audit.json').read_text())
        if cache_audit['complete']:
            notice = '<section id="historical-ir-cache"><h2>已确认：旧全量 IR 也包含缓存问题</h2><div class="note"><p>下面旧统计与论文对比图使用含缓存缺陷的原版 IR。已逐例追踪全部 418 个旧 IR 失败，并用修复版并行复验：Group 的 160 个全部恢复；Common 的 258 个中恢复 13 个，其中 21 个确认触发错误不可达。</p><p>173 个新成功轨迹已独立复核。原成功的 9,342 个没有用修复版重跑，因此本次核查不构成修复版全量成功率。旧成功轨迹合法，但缓存可能影响目标选择、SOC 和时间。</p><a href="cache_bug_review/index.html">查看完整核查、逐测例结果与真实日志</a></div></section>'
            if (full_fixed / 'completion_audit.json').exists() and json.loads((full_fixed / 'completion_audit.json').read_text())['complete']:
                notice = notice.replace('173 个新成功轨迹已独立复核。原成功的 9,342 个没有用修复版重跑，因此本次核查不构成修复版全量成功率。', '这里保留的是此前只核查 418 个失败记录的诊断结果。随后完成的 9,760 个实例全量重跑见上方修复版结果。')
            page = page.replace('<div class="cards">', notice + '<div class="cards">', 1)
            page = page.replace('<td>IR-TAPF</td>', '<td>IR-TAPF（原版缓存）</td>')
            page = page.replace('<h2>与论文一致的图形定义</h2>', '<h2>原版 IR：与论文一致的图形定义</h2>')
    if extension:
        page = page.replace('<section><h2>成功率：论文、原批次与复核后</h2>', extension + '<section><h2>成功率：论文、原批次与复核后</h2>', 1)
        page = page.replace('<h1>论文与复核结果，逐图对比</h1>', '<h1>论文与复核结果，逐图对比</h1><p><a href="#group-extension">新增：Group 250–500 agents 与 IR 缓存修复对照</a></p>', 1)
    (run / 'comparison.html').write_text(page)
    print('Comparison webpage:', run / 'comparison.html')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    build(args.run_dir.resolve())
