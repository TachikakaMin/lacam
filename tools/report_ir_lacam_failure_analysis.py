#!/usr/bin/env python3
"""Present completed controls and true search snapshots without new solver runs."""
import os,json,csv,hashlib,html
from pathlib import Path
os.environ['OPENBLAS_NUM_THREADS']='1'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
repo=Path(__file__).resolve().parents[1];root=repo/'build/results/group_250_500_cache_comparison/lacam_failure_analysis';pairs=json.loads((root.parent/'ir_success_lacam_failure.json').read_text())
variants={'original_replay':'C++ 原版重跑','cpp_initial_fixed':'C++ 固定自身首分配','ir_initial_fixed':'C++ 固定 IR 首分配','revisit_original_replay':'C++ 动态分配＋重访置顶','rust_ir_initial_fixed':'Rust 固定相同 IR 首分配','revisit_ir_initial_fixed':'C++ 固定 IR 首分配＋重访置顶（仅 7 例）'}
records=[];input_errors=[]
for pair in pairs:
 l=pair['lacam'];ir=pair['ir'];case=l['case_id']
 assert l['fixture_sha256']==ir['fixture_sha256'] and l['matrix_sha256']==ir['matrix_sha256'] and l['map_sha256']==ir['map_sha256']
 assert not l['valid_solution'] and ir['valid_solution'] and l['timed_out'] and ir['initial_solution_time_ms']<=10000
 assert hashlib.sha256(Path(l['fixture_file']).read_bytes()).hexdigest()==l['fixture_sha256']
 row={'case_id':case,'map':l['map'],'agents':l['num_agents'],'seed':l['seed'],'IR_full_first_ms':ir['initial_solution_time_ms']}
 for v in variants:
  p=root/case/v/'result.json'
  if p.exists():
   r=json.loads(p.read_text());row[v+'_success']=int(r['valid_solution_verified']);row[v+'_first_ms']=r.get('solve_time_ms',r.get('first_solution_time_ms')) if r['valid_solution_verified'] else None
 records.append(row)
fields=list(dict.fromkeys(k for r in records for k in r))
with (root/'case_controls.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(records)
counts={v:{'tested':sum(v+'_success' in r for r in records),'passed':sum(r.get(v+'_success',0) for r in records)} for v in variants}
summary={'cases':25,'same_input_hashes':True,'IR_final_archives_independently_validated':True,'all_original_lacam_failures_report_timeout':True,'controls':counts,'benchmark_results_unchanged':True,'interpretation':'Dynamic assignment and revisiting behavior affect finite-time success; completeness failure is not established.'}
(root/'analysis_summary.json').write_text(json.dumps(summary,indent=2))
trace=[json.loads(x) for x in (root/'symbotic_500_test6_search.jsonl').read_text().splitlines()]
fig,axes=plt.subplots(1,2,figsize=(11,3.8));t=[r['elapsed_ms']/1000 for r in trace]
axes[0].plot(t,[r['h'] for r in trace]);axes[0].set(xlabel='Elapsed search time (s)',ylabel='Assignment distance lower bound',yscale='log');axes[1].plot(t,[r['assigned_reached'] for r in trace]);axes[1].axhline(500,color='gray',linestyle='--');axes[1].set(xlabel='Elapsed search time (s)',ylabel='Agents at assigned targets',ylim=(0,515))
for ax in axes:ax.grid(alpha=.25)
fig.suptitle('True C++ dynamic-assignment search snapshots: symbotic, 500 agents, seed 6');fig.tight_layout();fig.savefig(root/'search_progress.png',dpi=170);fig.savefig(root/'search_progress.pdf');plt.close(fig)
source=Path(pairs[0]['lacam']['fixture_file']).parent/'symbotic.map';(root/'symbotic.map').write_bytes(source.read_bytes())
rows=''
for r in records:
 case=r['case_id'];cells=''.join('<td>'+('—' if v+'_success' not in r else ('✓ '+str(round(r[v+'_first_ms'],1))+' ms' if r[v+'_success'] else '× 10 s'))+'</td>' for v in variants)
 rows+='<tr><td>'+html.escape(case)+'</td><td>'+str(r['IR_full_first_ms'])+' ms</td>'+cells+'<td><a href="'+case+'/original_replay/result.json">重跑日志</a> · <a href="'+case+'/assignment_comparison.json">初始分配</a></td></tr>'
count_rows=''.join('<tr><td>'+variants[v]+'</td><td>'+str(c['passed'])+'/'+str(c['tested'])+'</td></tr>' for v,c in counts.items())
page='''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>IR 成功而 LaCAM 失败：控制实验</title><style>body{font:16px system-ui;max-width:1500px;margin:32px auto;padding:16px;line-height:1.6}table{border-collapse:collapse;font-size:13px}td,th{padding:8px;border:1px solid #ddd}img{max-width:100%}.scroll{overflow:auto}canvas{max-width:100%;border:1px solid #ddd}.flow{display:flex;gap:14px;flex-wrap:wrap}.flow div{padding:16px;background:#eef3f8;border-radius:8px}input[type=range]{width:60%}pre{white-space:pre-wrap}button{padding:8px}</style><h1>为什么 IR 成功，而 LaCAM 失败？</h1><p><strong>输入一致，IR 轨迹核查通过。有限时间内的搜索行为不同。</strong>这里的 IR 指缓存修复版。共 25 个反例：同一地图、起点、候选目标、测例种子，10 秒搜索预算。IR 内部用的是 Rust 固定目标 LaCAM；当前 TAPF 分支用的是 C++ 动态目标 LaCAM，两个实现并不等价。</p><div class="flow"><div>IR：初始分配 → 固定目标<br>Rust LaCAM 重访已知节点 → 路径</div><div>C++ TAPF：产生新节点 → 动态重分配<br>已在 OPEN 的已知节点不重新置顶 → 继续搜索</div></div><h2>控制实验结果</h2><table><tr><th>处理</th><th>成功 / 测试数</th></tr>'''+count_rows+'''</table><p>冻结目标的实验只保留原候选集中的一个目标，地图、起点、agent 数不变。使用的是搜索开始前的首次分配，未使用 IR 优化后的最终目标。成功轨迹按原任务独立核查。Rust 对照还按完全相同的单目标任务核查。</p><p>最后一行只测试“固定 IR 目标后 C++ 仍失败”的 7 例，不能把它当作修改版全量成功率。每个控制各运行一次；诊断结果未合并进主实验统计。</p><h2>两个可观察的原因</h2><p><strong>1. 动态重分配会改变搜索方向。</strong>仅冻结 C++ 自己的首次分配，15 个反例便能成功。symbotic / 500 / seed 6 的动态搜索生成约 34,000 个节点，几乎每个节点都换分配，后半段距离下界停留在约 430，10 秒仍未完成。冻结自身首分配后，2.628 秒找到解；冻结 IR 首分配后，2.354 秒找到解。</p><p><strong>2. 已知节点的重访处理与原始 LaCAM 不同。</strong>原始 C++ LaCAM 和 IR 的 Rust LaCAM 会将已知节点放到搜索栈顶。当前 TAPF 分支用 queued 标记去重：节点已在 OPEN 时，不重新置顶。诊断版只恢复重访置顶，动态分配仍保留，17/25 个反例成功。这是有限时间性能差异的证据，尚不能据此断言算法不完备。</p><p>固定完全相同的 IR 首分配时，C++ 为 18/25，Rust 为 25/25。其余 7 例中，恢复 C++ 重访置顶又解出 6 例。随机数实现、候选动作排序等仍有差异；剩余一例的单独原因尚未确定。</p><p><a href="revisit_control.patch">重访控制补丁</a> · <a href="case_controls.csv">逐测例控制结果 CSV</a> · <a href="analysis_summary.json">核查摘要</a> · <a href="../comparison.html">主实验图与统计</a></p><h2>真实搜索快照：symbotic / 500 / seed 6</h2><img src="search_progress.png"><p>下图是实际搜索节点快照，不是连续执行路径。拖动可看目标与位置变化。节点深度表示搜索路径长度，不表示执行用时。</p><button id="play">播放</button><input id="frame" type="range" min="0" value="0"><label>查看 agent <input id="agent" type="number" value="0" min="0" max="499" style="width:65px"></label><pre id="metrics"></pre><canvas id="board" width="740" height="780"></canvas><p>绿色：位于当前分配目标；橙色：尚未到达。蓝色是选中的 agent，紫色方框是它的当前目标。</p><h2>25 个反例明细</h2><input id="filter" placeholder="输入地图、规模或 seed 筛选"><div class="scroll"><table id="cases"><thead><tr><th>测例</th><th>IR 完整算法首解</th>'''+''.join('<th>'+html.escape(v)+'</th>' for v in variants.values())+'''<th>证据</th></tr></thead><tbody>'''+rows+'''</tbody></table></div><script>
let snapshots=[],map=[],timer=null;const board=document.getElementById('board'),ctx=board.getContext('2d'),slider=document.getElementById('frame');
function draw(){if(!snapshots.length)return;const s=snapshots[Number(slider.value)],chosen=Math.max(0,Math.min(499,Number(document.getElementById('agent').value)));const cell=20;ctx.clearRect(0,0,board.width,board.height);map.forEach((row,y)=>Array.from(row).forEach((c,x)=>{ctx.fillStyle=c==='.'?'#f5f7fa':'#555';ctx.fillRect(x*cell,y*cell,cell,cell)}));s.positions.forEach((p,i)=>{const g=s.targets[i];ctx.fillStyle=i===chosen?'#2469d8':(p[0]===g[0]&&p[1]===g[1]?'#2d9a68':'#e99225');ctx.beginPath();ctx.arc(p[1]*cell+10,p[0]*cell+10,i===chosen?7:4,0,Math.PI*2);ctx.fill()});const p=s.positions[chosen],g=s.targets[chosen];ctx.strokeStyle='#8d43be';ctx.lineWidth=2;ctx.strokeRect(g[1]*cell+3,g[0]*cell+3,14,14);ctx.beginPath();ctx.moveTo(p[1]*cell+10,p[0]*cell+10);ctx.lineTo(g[1]*cell+10,g[0]*cell+10);ctx.stroke();document.getElementById('metrics').textContent=JSON.stringify({snapshot:Number(slider.value)+1,total:snapshots.length,elapsed_ms:s.elapsed_ms,nodes:s.nodes,depth:s.depth,distance_lower_bound:s.h,assignment_changes:s.assignment_changes,agents_at_assigned_targets:s.assigned_reached,selected_agent:chosen,position:p,target:g},null,2)}
Promise.all([fetch('symbotic_500_test6_search.jsonl').then(x=>x.text()),fetch('symbotic.map').then(x=>x.text())]).then(([t,m])=>{snapshots=t.trim().split('\n').map(JSON.parse);map=m.trimEnd().split('\n').slice(4);slider.max=snapshots.length-1;draw()});slider.oninput=draw;document.getElementById('agent').oninput=draw;document.getElementById('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;return}timer=setInterval(()=>{slider.value=(Number(slider.value)+1)%snapshots.length;draw()},250)};document.getElementById('filter').oninput=e=>{for(const r of document.querySelectorAll('#cases tbody tr'))r.hidden=!r.textContent.toLowerCase().includes(e.target.value.toLowerCase())};
</script></html>'''
(root/'analysis.html').write_text(page)
comparison=root.parent/'comparison.html';text=comparison.read_text();link='<p><a href="lacam_failure_analysis/analysis.html"><strong>IR 成功而 LaCAM 失败：25 个反例的控制实验与真实搜索回放</strong></a></p>'
if 'lacam_failure_analysis/analysis.html' not in text:text=text.replace('<h2>修复前后：同一批输入</h2>',link+'<h2>修复前后：同一批输入</h2>',1);comparison.write_text(text)
print(json.dumps(summary,ensure_ascii=False))
