#!/usr/bin/env python3
"""Publish measured dynamic-TA diagnostics; never merge different solvers' successes."""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
import csv
import hashlib
import html
import json
import statistics
from collections import Counter
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml

repo = Path(__file__).resolve().parents[1]
root = repo / 'build/results/ita_optimization_500'
pairs = json.loads((root/'all_pairs.json').read_text())
screen = json.loads((root/'dynamic_queue_screen/results.json').read_text())
extension = json.loads((root/'extension_paired/results.json').read_text())
scope = json.loads((root/'input_scope_verification.json').read_text())
ir_audit = json.loads((root/'ir_counterexample_archive_verification.json').read_text())
assert scope['all_cases'] == scope['unique_case_ids'] == 10720
assert not scope['input_hash_errors'] and not ir_audit['errors'] and ir_audit['checked'] == 28
assert scope['checked_fixture_matrix_map_on_disk'] and not scope['matrix_map_disk_hash_errors']
assert len(screen) == 540 and len(extension) == 1920
assert not any('infrastructure_error' in r for r in screen + extension)
labels = {'baseline':'原版重跑', 'restart_only':'仅修复重启', 'restart_fast':'重启修复＋等价计算优化', 'native_stack':'恢复旧节点入栈', 'stack_fast':'旧节点入栈＋计算优化'}
by_screen = {(r['case_id'],r['variant']):r for r in screen}
by_extension = {(r['case_id'],r['variant']):r for r in extension}
assert len(by_screen) == 540 and len(by_extension) == 1920
ces = [p for p in pairs if p['any_ir_counterexample']]
trace_summary = Counter()
case_rows = []
maps = {}
for p in sorted(ces, key=lambda p:(p['lacam']['map'],int(p['lacam']['num_agents']),int(p['lacam']['seed']))):
    l = p['lacam']; case = p['case_id']; row = {'case_id':case, 'map':l['map'], 'agents':int(l['num_agents']), 'seed':int(l['seed']), 'ir_source':'修复后 IR' if p['fixed_counterexample'] else '仅旧版 IR 有已验证解'}
    ir = p['fixed_ir'] if p['fixed_counterexample'] else p['original_ir']
    row['ir_first_ms'] = float(ir['initial_solution_time_ms'])
    row['ir_soc'] = int(float(ir['soc']))
    for v in labels:
        record = by_screen[case,v]
        row[v] = {k:record.get(k) for k in ['valid_solution_verified','first_solution_time_ms','soc','assignment_changes','hl_nodes_created']}
    for v in ['baseline','restart_fast']:
        trace_file = root/'dynamic_traces'/(v+'_trace')/case/'search.jsonl'
        snapshots = [json.loads(line) for line in trace_file.read_text().splitlines()]
        last = snapshots[-1]
        row[v+'_trace'] = {k:last[k] for k in ['h','assigned_reached','restart_selected','restart_applied','restart_blocked','duplicate_queued','assignment_changes','nodes']}
        row[v+'_extension'] = by_extension[case,v]['valid_solution_verified']
        for k in ['restart_selected','restart_applied','restart_blocked','duplicate_queued']:
            trace_summary[v+'_'+k] += last[k]
    row['diagnosis'] = ('重启修复后成功' if row['restart_fast']['valid_solution_verified'] else
                        '10 秒内未触发重启' if row['restart_fast_trace']['restart_selected']==0 else '重启生效后仍超时')
    case_rows.append(row)
    if l['map'] not in maps:
        fixture_path = Path(l['fixture_file'])
        matching = [f for f in fixture_path.parent.glob('*.map') if hashlib.sha256(f.read_bytes()).hexdigest()==l['map_sha256']]
        assert len(matching) == 1
        grid = matching[0].read_text().splitlines()[4:]
        maps[l['map']] = {'dimensions':[len(grid),len(grid[0])], 'obstacles':[[y,x] for y,row in enumerate(grid) for x,ch in enumerate(row) if ch in '@T']}

paired = {}
for v in ['baseline','restart_fast']:
    rows = [r for r in extension if r['variant']==v]
    paired[v] = {'cases':len(rows), 'solved':sum(r['valid_solution_verified'] for r in rows), 'counterexamples_solved':sum(r['valid_solution_verified'] and r['any_ir_counterexample'] for r in rows)}
improvements = []; regressions = []; common = []
for p in pairs:
    case = p['case_id']
    if (case,'baseline') not in by_extension: continue
    b,a = by_extension[case,'baseline'],by_extension[case,'restart_fast']
    if a['valid_solution_verified'] and not b['valid_solution_verified']: improvements.append(case)
    if b['valid_solution_verified'] and not a['valid_solution_verified']: regressions.append(case)
    if a['valid_solution_verified'] and b['valid_solution_verified']: common.append((b,a))
summary = {'scope':{k:v for k,v in scope.items() if k!='coverage'},'screen':json.loads((root/'dynamic_queue_screen/summary.json').read_text()), 'extension':paired, 'paired_recovered':improvements, 'paired_regressions':regressions, 'common_solved':len(common), 'median_first_time_ratio':statistics.median(a['first_solution_time_ms']/max(b['first_solution_time_ms'],.001) for b,a in common), 'median_soc_ratio':statistics.median(a['soc']/b['soc'] for b,a in common), 'trace_counters':dict(trace_summary),'default_solver_changed_to_candidate':False,'dynamic_ta_preserved':True,'formal_benchmark_statistics_changed':False}
(root/'analysis_summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
(root/'counterexample_details.json').write_text(json.dumps(case_rows,indent=2,ensure_ascii=False))
(root/'trace_maps.json').write_text(json.dumps(maps))
fields = ['case_id','variant','map','num_agents','seed','valid_solution_verified','first_solution_time_ms','soc','hl_nodes_created','assignment_changes','hl_duplicate_configs','hl_reinsertions','assignment_time_ms','fixture_sha256','binary_sha256']
for name,records in [('screen_results',screen),('extension_results',extension)]:
    with (root/(name+'.csv')).open('w') as f:
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(records)

# Paper-like success-rate panels. IR is the historical cache-fixed run, explicitly labelled.
map_names = sorted({r['map'] for r in extension})
fig,axes = plt.subplots(2,4,figsize=(15,7),sharex=True,sharey=True)
for ax,m in zip(axes.flat,map_names):
    for variant,color,label in [('baseline','#cc3333','ITA-LaCAM baseline'),('restart_fast','#7645ad','ITA-LaCAM restart fix')]:
        points = [(n,[r for r in extension if r['map']==m and r['num_agents']==n and r['variant']==variant]) for n in [250,300,350,400,450,500]]
        ax.plot([n for n,rs in points],[100*sum(r['valid_solution_verified'] for r in rs)/len(rs) for n,rs in points],'-o',color=color,label=label,markersize=4)
    ir_points = [[p['fixed_ir'] for p in pairs if p['lacam']['map']==m and int(p['lacam']['num_agents'])==n] for n in [250,300,350,400,450,500]]
    ax.plot([250,300,350,400,450,500],[100*sum(int(r['valid_solution']) for r in rs)/len(rs) for rs in ir_points],'--s',color='#2a9855',label='IR-TAPF (historical)',markersize=3)
    ax.set_title(m);ax.set_ylim(-3,103);ax.grid(alpha=.2)
for ax in axes[1]:ax.set_xlabel('Number of agents')
for ax in axes[:,0]:ax.set_ylabel('Success rate (%)')
handles,legend=axes[0,0].get_legend_handles_labels();fig.legend(handles,legend,loc='upper center',ncol=3)
fig.tight_layout(rect=(0,0,1,.94))
for ext in ['png','pdf','svg']:fig.savefig(root/('success_250_500.'+ext),dpi=170)
plt.close(fig)

counts=summary['screen']; table=''
for v in labels:
    c=counts[v]
    table+=f'<tr><td>{labels[v]}</td><td>{c["counterexamples"]["solved"]}/28</td><td>{c["original_failures"]["solved"]}/54</td><td>{c["success_controls"]["solved"]}/54</td></tr>'
detail=''
for row in case_rows:
    cells=''.join('<td>'+('✓ '+str(round(row[v]['first_solution_time_ms'],1))+' ms' if row[v]['valid_solution_verified'] else '超时')+'</td>' for v in ['baseline','restart_only','restart_fast','native_stack'])
    detail+=f'<tr><td><button class="case" data-case="{html.escape(row["case_id"])}">{html.escape(row["case_id"].split("/")[-1])}</button></td><td>{row["ir_source"]}<br>{row["ir_first_ms"]:.1f} ms</td>{cells}<td>{row["diagnosis"]}</td><td><a href="dynamic_queue_screen/restart_fast/{row["case_id"]}/result.json">结果</a> · <a href="dynamic_traces/baseline_trace/{row["case_id"]}/search.jsonl">原版轨迹</a></td></tr>'
regression_links=''.join(f'<li><a href="extension_paired/restart_fast/{c}/result.json">{html.escape(c)}</a></li>' for c in regressions)
page=r'''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>ITA-LaCAM：保留动态 TA 的失败分析</title>
<style>body{font:16px system-ui;line-height:1.65;margin:0;color:#233046;background:#f4f7fb}main{max-width:1450px;margin:auto;padding:28px}section,.hero{background:white;padding:24px;border-radius:12px;margin:18px 0}h1{font-size:28px}h2{font-size:22px}.cards,.twocol{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:18px}.cards{grid-template-columns:repeat(4,1fr)}.card{background:#eef3fa;padding:16px;border-radius:8px}.card strong{display:block;font-size:27px}.note{background:#fff5d6;padding:14px;border-left:4px solid #cc9500}table{border-collapse:collapse;width:100%;font-size:14px}td,th{padding:9px;border-bottom:1px solid #dbe2ec;text-align:left}th{background:#edf2f9}.scroll{overflow:auto}button,select,input{font:inherit;padding:6px;border:1px solid #bfc9da;border-radius:5px}button{cursor:pointer;background:white;color:#215e9d}.case{text-align:left;font-size:13px}img,canvas,svg{max-width:100%}pre{white-space:pre-wrap;font-size:13px;background:#f4f7fb;padding:12px}a{color:#245fab}.flow{display:flex;flex-wrap:wrap;align-items:center;gap:10px}.flow span{padding:10px;background:#edf2f9;border-radius:5px}#time{width:50%}.metric{min-height:135px}canvas{background:#eee}small{color:#59667a}@media(max-width:800px){.twocol,.cards{grid-template-columns:1fr}main{padding:12px}}</style>
<main><div class="hero"><a href="../comparison.html">← 原统计与论文对比</a><h1>ITA-LaCAM：保留动态 TA，修复无效重启</h1><p>检查到 500 agent 的全部输入。只改搜索队列及冗余计算。每个新配置的动态 TA 结果继续用于目标、优先级与 PIBT。</p><div class="cards"><div class="card"><strong>10,720</strong>唯一用例，输入哈希一致</div><div class="card"><strong>25 + 3</strong>IR 有有效解而原 LaCAM 超时</div><div class="card"><strong>18 / 28</strong>仅修复重启后的反例成功数</div><div class="card"><strong>570 / 570</strong>原版轨迹中被阻止的重启</div></div></div>
<section><h2>问题发生在哪里</h2><div class="flow"><span>再次遇到已知配置</span>→<span>随机选中初始节点重启</span>→<span>初始节点已在 OPEN：queued = true</span>→<span>!queued 条件失败</span>→<span>未放到栈顶，重启未执行</span></div><p>小改动：选中重启时，将初始节点已有的 OPEN 项移到栈顶。保留原有重启概率 0.001，保留原有动态 TA 调用与返回值。普通旧节点仍按原逻辑处理。</p><p><a href="variants/restart_only/change.patch">仅重启修复补丁</a> · <a href="variants/restart_fast/change.patch">含计算优化的完整补丁</a> · <a href="variants/restart_fast/tapf_planner.cpp">实验源代码</a> · <a href="original/tapf_planner.cpp">原版源代码</a></p><p>28 例真实轨迹：原版选中重启 570 次，生效 0 次；修复版选中 91 次，生效 91 次。轨迹运行单独计数，不并入正式性能实验。这个缺陷解释了部分有限时间失败；没有证明它是全部失败的唯一原因。</p><p>IR 的内层 MAPF 搜索使用该轮已选定的目标；ITA-LaCAM 在搜索新配置时更新目标。搜索顺序不同，允许更多目标分配并不保证在相同的 10 秒内更早找到解。这不是取消动态 TA 的理由。</p><p>原始 LaCAM 在遇到已知配置时会再次入栈。恢复这一行为也能改善反例，但成功对照退化更多。当前证据更支持优先修复原有重启。</p></section>
<section><h2>独立对照：54 个原失败例 + 54 个成功例</h2><p>每次 10 秒、相同输入与 seed。56 个独立物理核并行。每条成功路径另进程检查目标兼容、移动、碰撞和 SOC。每列是一套固定版本的结果，不拼接不同版本的成功例。</p><table><thead><tr><th>版本</th><th>28 个 IR 反例</th><th>54 个原失败例</th><th>54 个原成功例</th></tr></thead><tbody>__SCREEN__</tbody></table><p>等价计算优化：DFS 不计算未使用的 FOCAL 指标；目标检查由逐目标扫描改为索引查找。它们不覆盖动态 TA，也不主动改变搜索顺序。</p><p><a href="screen_results.csv">540 次结果 CSV</a> · <a href="dynamic_queue_screen/provenance.json">执行参数与二进制哈希</a></p></section>
<section><h2>扩展验证：250–500 agent 的全部 960 例</h2><p>原版重跑 <strong>__BASE__/960</strong>；重启修复＋计算优化 <strong>__NEW__/960</strong>。相对同轮原版，新增成功 <strong>__GAIN__</strong> 例，新增失败 <strong>__LOSS__</strong> 例。</p><p>双方共同成功 __COMMON__ 例；修复版/原版的首解耗时比中位数 __TIME__，SOC 比中位数 __SOC__。比值小于 1 表示更低。只比较共同成功用例，不能把失败例记为零耗时或零代价。</p><div class="note">候选修复尚未替换默认求解器，也没有改写原正式统计。仍有退化例，不能据此宣称所有规模都稳定提升。曲线中 IR 为历史修复版结果，未在本轮重跑，硬件负载并非完全相同。</div><img src="success_250_500.png" alt="8 个地图上 250 至 500 agent 的成功率对比"><p><a href="success_250_500.pdf">PDF</a> · <a href="success_250_500.svg">SVG</a> · <a href="extension_results.csv">1,920 次完整结果</a> · <a href="extension_paired/provenance.json">成对验证参数</a></p><details><summary>查看全部新增失败例</summary><ul>__REGRESSIONS__</ul></details></section>
<section><h2>28 个反例：真实搜索状态对照</h2><p>点击下方任一用例切换。时间轴显示实际搜索节点快照，<strong>不是可执行路径</strong>。每 1,000 个新节点采样，并记录首解。重启事件单独留在原始日志，动画不把未执行的重启目标当成已访问状态。轨迹插桩有开销；成功率以不插桩的实验表为准。</p><select id="caseSelect"></select><p><button id="play">播放 / 暂停</button> <input id="time" type="range" min="0" max="10000" value="0" step="10"> <span id="clock">0.00 s</span>　agent <input id="agent" type="number" min="0" value="0" style="width:70px"></p><p>绿点：已到当前分配目标；橙点：未到；蓝点：所选 agent；紫框：它的当前动态目标。</p><div class="twocol"><div><h3>原版</h3><canvas id="before" width="640" height="640"></canvas><pre class="metric" id="beforeMetrics"></pre></div><div><h3>重启修复＋计算优化</h3><canvas id="after" width="640" height="640"></canvas><pre class="metric" id="afterMetrics"></pre></div></div><svg id="progress" viewBox="0 0 1100 230" role="img" aria-label="两版搜索距离下界随时间变化"></svg><p id="diagnosis"></p><p><small>距离下界 h 是当前配置的最小分配距离和，不含 agent 之间的避碰代价。h 很小仍可能难解。结束摘要取全程最小 h 节点，不能解读为最终执行位置。</small></p></section>
<section><h2>逐例结果与证据</h2><input id="filter" placeholder="筛选地图、agent 数、seed"><div class="scroll"><table id="cases"><thead><tr><th>用例（点击回放）</th><th>IR 有效解来源与首解</th><th>原版重跑</th><th>仅修复重启</th><th>重启＋计算优化</th><th>恢复旧节点入栈</th><th>轨迹观察</th><th>证据</th></tr></thead><tbody>__DETAIL__</tbody></table></div><p>25 例使用修复后 IR 的有效解；额外 3 例只在旧版 IR 留有有效解。28 条归档路径均重新核验。所有原 LaCAM 失败例都报告超时；“no solution found”在这些日志中不表示数学上无解。</p><p><a href="trace_state_validation.json">56 次轨迹状态核验</a> · <a href="dynamic_ta_code_audit.json">动态 TA 代码核查</a> · <a href="input_scope_verification.json">全部输入覆盖与哈希</a> · <a href="ir_counterexample_archive_verification.json">IR 归档复核</a> · <a href="analysis_summary.json">完整统计摘要</a> · <a href="counterexample_details.json">逐例诊断 JSON</a></p></section></main>
<script>
let rows=[],maps={},traces=[],current=null,timer=null,loadId=0;
const $=id=>document.getElementById(id), slider=$('time');
function drawMap(canvasId,metricId,records){
 const t=+slider.value, snapshots=records.filter(x=>['initial','sample','solution'].includes(x.event));let s=snapshots[0];for(const x of snapshots){if(x.elapsed_ms<=t)s=x;else break;}
 const map=maps[current.map], dims=map.dimensions, c=$(canvasId),ctx=c.getContext('2d'),cell=Math.min(640/dims[1],640/dims[0]);ctx.clearRect(0,0,640,640);ctx.fillStyle='#edf0f4';ctx.fillRect(0,0,dims[1]*cell,dims[0]*cell);ctx.fillStyle='#536170';for(const p of map.obstacles){ctx.fillRect(p[1]*cell,p[0]*cell,cell,cell);}
 const chosen=Math.max(0,Math.min(s.positions.length-1,+$('agent').value||0));
 s.positions.forEach((p,i)=>{let g=s.targets[i];ctx.fillStyle=i===chosen?'#1265ed':p[0]===g[0]&&p[1]===g[1]?'#269a60':'#e4982c';ctx.beginPath();ctx.arc((p[1]+.5)*cell,(p[0]+.5)*cell,cell*(i===chosen?.38:.23),0,Math.PI*2);ctx.fill();});
 let p=s.positions[chosen],g=s.targets[chosen];ctx.strokeStyle='#8b39c3';ctx.lineWidth=2;ctx.strokeRect(g[1]*cell+2,g[0]*cell+2,cell-4,cell-4);ctx.beginPath();ctx.moveTo((p[1]+.5)*cell,(p[0]+.5)*cell);ctx.lineTo((g[1]+.5)*cell,(g[0]+.5)*cell);ctx.stroke();
 $(metricId).textContent=`快照 ${s.elapsed_ms.toFixed(1)} ms · ${s.event}\nh=${s.h} · 到当前目标 ${s.assigned_reached}/${s.positions.length}\n新节点 ${s.nodes} · 深度 ${s.depth} · TA 变化节点 ${s.assignment_changes}\n已触发重启 ${s.restart_selected} · 生效 ${s.restart_applied} · 被阻止 ${s.restart_blocked}\nagent ${chosen}: [${p}] → [${g}]`;
}
function draw(){if(!current||!traces.length)return;$('clock').textContent=(+slider.value/1000).toFixed(2)+' s';drawMap('before','beforeMetrics',traces[0]);drawMap('after','afterMetrics',traces[1]);}
function chart(){const colors=['#cc3333','#7645ad'];let max=Math.max(...traces.flat().map(x=>x.h)),parts=['<rect width="1100" height="230" fill="#f4f7fb"/>'];traces.forEach((list,i)=>{let best=Infinity;let pts=list.filter(x=>['initial','sample','solution'].includes(x.event)).map(x=>{best=Math.min(best,x.h);return `${50+Math.min(x.elapsed_ms,10000)/10.0},${185-155*Math.log1p(best)/Math.log1p(max)}`}).join(' ');parts.push(`<polyline fill="none" stroke="${colors[i]}" stroke-width="2" points="${pts}"/>`)});parts.push('<text x="50" y="20" font-size="15">已采样节点的最小距离下界 h（log(1+h)）</text><text x="50" y="215">0 s</text><text x="1000" y="215">10 s</text><text x="740" y="20" fill="#cc3333">原版</text><text x="830" y="20" fill="#7645ad">重启修复＋计算优化</text>');$('progress').innerHTML=parts.join('');}
async function choose(id){const token=++loadId;current=rows.find(x=>x.case_id===id);$('caseSelect').value=id;$('agent').max=current.agents-1;const loaded=await Promise.all(['baseline_trace','restart_fast_trace'].map(v=>fetch(`dynamic_traces/${v}/${id}/search.jsonl`).then(r=>{if(!r.ok)throw Error(r.status);return r.text()}).then(t=>t.trim().split('\n').map(JSON.parse))));if(token!==loadId)return;traces=loaded;slider.value=10000;$('diagnosis').textContent=`${current.case_id}：${current.diagnosis}。全程最小 h：原版 ${current.baseline_trace.h}，修复版 ${current.restart_fast_trace.h}。`;draw();chart();}
Promise.all([fetch('counterexample_details.json').then(r=>r.json()),fetch('trace_maps.json').then(r=>r.json())]).then(([a,b])=>{rows=a;maps=b;rows.forEach(r=>{let o=document.createElement('option');o.value=r.case_id;o.textContent=r.case_id.split('/')[1];$('caseSelect').appendChild(o)});return choose(rows.find(r=>r.case_id.includes('symbotic_agents_300_test_19')).case_id)}).catch(e=>{$('diagnosis').textContent='载入失败：'+e;throw e});
$('caseSelect').onchange=e=>choose(e.target.value);slider.oninput=draw;$('agent').oninput=draw;
$('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;return}if(+slider.value>=10000)slider.value=0;timer=setInterval(()=>{slider.value=Math.min(10000,+slider.value+100);draw();if(+slider.value>=10000){clearInterval(timer);timer=null}},100)};
document.querySelectorAll('button.case').forEach(b=>b.onclick=()=>{choose(b.dataset.case);$('caseSelect').scrollIntoView({behavior:'smooth'})});$('filter').oninput=e=>{document.querySelectorAll('#cases tbody tr').forEach(r=>r.hidden=!r.textContent.toLowerCase().includes(e.target.value.toLowerCase()))};
</script></html>'''
tokens={'__SCREEN__':table,'__DETAIL__':detail,'__BASE__':paired['baseline']['solved'],'__NEW__':paired['restart_fast']['solved'],'__GAIN__':len(improvements),'__LOSS__':len(regressions),'__COMMON__':len(common),'__TIME__':f'{summary["median_first_time_ratio"]:.3f}','__SOC__':f'{summary["median_soc_ratio"]:.3f}','__REGRESSIONS__':regression_links}
for k,v in tokens.items(): page=page.replace(k,str(v))
(root/'analysis.html').write_text(page)
web = repo/'build/results/paper_2609_14208_repository'
link=web/'ita_dynamic_analysis'
if not link.exists():link.symlink_to(root,target_is_directory=True)
main=web/'comparison.html';text=main.read_text()
marker='id="ita-dynamic-analysis-link"'
if marker not in text:
    import re
    text=re.sub(r'(<body[^>]*>)',r'\1'+'<p '+marker+' style="padding:16px;background:#eef3ff"><a href="ita_dynamic_analysis/analysis.html"><strong>新增：保留动态 TA 的 ITA-LaCAM 失败分析、960 例验证与真实搜索回放</strong></a></p>',text,count=1)
    assert marker in text
    main.write_text(text)
print(json.dumps({k:v for k,v in summary.items() if k not in ['screen','scope','paired_recovered','paired_regressions']},ensure_ascii=False,indent=2))
