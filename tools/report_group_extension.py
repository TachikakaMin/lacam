#!/usr/bin/env python3
"""Audit and plot the native-generator Group 250–500 experiment."""
import argparse, concurrent.futures, hashlib, json, os, time
from collections import Counter
from pathlib import Path
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):
 os.environ[key]='1'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
from report_paper_2609_14208 import verify_archived_schedule
from run_paper_2609_14208 import atomic_json

p=argparse.ArgumentParser()
p.add_argument('--run-dir',type=Path,required=True)
p.add_argument('--manifest',type=Path,required=True)
a=p.parse_args(); run=a.run_dir.resolve(); manifest=json.loads(a.manifest.read_text())
rows=[json.loads(p.read_text()) for p in (run/'artifacts').glob('*/*/*/result.json')]
expected={(c['case_id'],m) for c in manifest['cases'] for m in ('lacam_dfs','ir')}
actual={(r['case_id'],r['method']) for r in rows}
configuration_errors=[]
maps=list(dict.fromkeys(c['map'] for c in manifest['cases']))
if len(maps)!=8 or len(manifest['cases'])!=960:
 configuration_errors.append('expected 8 maps and 960 cases')
for name in maps:
 observed=Counter((c['num_agents'],c['seed']) for c in manifest['cases'] if c['map']==name)
 required=Counter({(n,seed):1 for n in (250,300,350,400,450,500) for seed in range(20)})
 if observed!=required: configuration_errors.append(name+': agent-count/seed coverage')
if any(c['suite']!='group' for c in manifest['cases']): configuration_errors.append('unexpected suite')
provenance=json.loads((run/'provenance.json').read_text())
reserved=set(provenance['reserved_cpus'])
os.sched_setaffinity(0,set(os.sched_getaffinity(0))-reserved)
if len(reserved)<4: configuration_errors.append('fewer than four reserved CPUs')
if set(provenance['binaries_sha256'])!={'lacam_dfs','ir'}: configuration_errors.append('unexpected solver set')
for row in rows:
 command=json.loads((run/'artifacts'/row['case_id']/row['method']/'command.json').read_text())['argv']
 if command[:2]!=['taskset','-c'] or int(command[2]) in reserved:
  configuration_errors.append(row['case_id']+': CPU affinity')
 if row['binary_sha256']!=provenance['binaries_sha256'].get(row['method']):
  configuration_errors.append(row['case_id']+': binary hash')
 if row['time_limit']!=10 or row.get('external_timed_out'):
  configuration_errors.append(row['case_id']+': time budget')
 if row['valid_solution']:
  first=row.get('first_solution_time_ms',row.get('initial_solution_time_ms',None))
  if not isinstance(first,(float,int)) or first>10000: configuration_errors.append(row['case_id']+': first solution time')

errors=[r for r in rows if r.get('infrastructure_error')]
audit={'expected_rows':len(expected),'actual_rows':len(rows),'missing_rows':len(expected-actual),
 'unexpected_rows':len(actual-expected),'duplicates':len(rows)-len(actual),'infrastructure_errors':len(errors),
 'configuration_errors':configuration_errors,'reserved_cpus':sorted(reserved)}
input_errors=[]
for c in manifest['cases']:
 for field in ('fixture','matrix'):
  if hashlib.sha256(Path(c[field+'_file']).read_bytes()).hexdigest()!=c[field+'_sha256']:
   input_errors.append(c['case_id']+'/'+field)
for c in manifest['cases']:
 lines=Path(c['matrix_file']).read_text().splitlines()
 map_path=Path(lines[1].split(':',1)[1].strip())
 if hashlib.sha256(map_path.read_bytes()).hexdigest()!=c['map_sha256']:
  input_errors.append(c['case_id']+'/map')
audit['input_hash_errors']=input_errors
success=[r for r in rows if r['valid_solution']]
cpus=sorted(set(os.sched_getaffinity(0))-reserved)[:24]
status=json.loads((run/'status.json').read_text())
status.update(state='auditing',report_pid=os.getpid(),archive_validation_total=len(success),archive_validation_completed=0)
atomic_json(run/'status.json',status)
archive_errors=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=len(cpus)) as pool:
 futures=[pool.submit(verify_archived_schedule,(run,r,cpus[i%len(cpus)])) for i,r in enumerate(success)]
 last_update=time.monotonic()
 for done,future in enumerate(concurrent.futures.as_completed(futures),1):
  error=future.result()
  if error: archive_errors.append(error)
  if time.monotonic()-last_update>=5 or done==len(futures):
   status.update(archive_validation_completed=done)
   atomic_json(run/'status.json',status)
   print(f'archive audit {done}/{len(futures)} errors={len(archive_errors)}',flush=True)
   last_update=time.monotonic()
audit['archived_solutions_checked']=len(success); audit['archive_errors']=archive_errors
audit['complete']=actual==expected and len(rows)==len(expected) and not errors and not input_errors and not archive_errors and not configuration_errors
atomic_json(run/'coverage_audit.json',audit)
df=pd.DataFrame(rows); df.to_csv(run/'rows.csv',index=False)
metrics=df.groupby(['map','num_agents','method']).agg(completed=('valid_solution','size'),solved=('valid_solution','sum')).reset_index()
metrics['success_rate']=100*metrics.solved/metrics.completed
metrics.to_csv(run/'success_rates.csv',index=False)
labels={'lacam_dfs':'ITA-LaCAM','ir':'IR-TAPF (cache fixed)' if provenance.get('ir_variant')=='distance_cache_fixed' else 'IR-TAPF (original cache)'}
colors={'lacam_dfs':'#d62728','ir':'#1f77b4' if provenance.get('ir_variant')=='distance_cache_fixed' else '#2ca02c'}
fig,axes=plt.subplots(2,4,figsize=(16,7),sharex=True,sharey=True)
for ax,mapname in zip(axes.flat,dict.fromkeys(c['map'] for c in manifest['cases'])):
 for method in labels:
  s=metrics[(metrics['map']==mapname)&(metrics.method==method)]
  ax.plot(s.num_agents,s.success_rate,'o-',label=labels[method],color=colors[method])
 ax.set(title=mapname,ylim=(-3,103),xticks=[250,300,350,400,450,500],xlabel='Number of agents',ylabel='Success rate (%)'); ax.grid(alpha=.25)
axes[0,0].legend(); fig.suptitle('Group size 5 (component remainders smaller), 20 instances per size, 10 s budget'); fig.tight_layout()
fig.savefig(run/'group_success.png',dpi=180); fig.savefig(run/'group_success.pdf'); plt.close(fig)
# Compare cost and first-solution time on exactly the same successful cases.
first_column={'lacam_dfs':'first_solution_time_ms','ir':'initial_solution_time_ms'}
paired=[]
by_key={(r['case_id'],r['method']):r for r in rows}
for c in manifest['cases']:
 l=by_key.get((c['case_id'],'lacam_dfs')); i=by_key.get((c['case_id'],'ir'))
 if l and i and l['valid_solution'] and i['valid_solution']:
  paired.append({'case_id':c['case_id'],'map':c['map'],'num_agents':c['num_agents'],'seed':c['seed'],
    'lacam_soc':l['soc'],'ir_soc':i['soc'],
    'soc_ratio_ir_over_lacam':i['soc']/l['soc'] if l['soc'] else 1,
    'lacam_first_ms':l['first_solution_time_ms'],'ir_first_ms':i['initial_solution_time_ms']})
pairs=pd.DataFrame(paired)
pairs.to_csv(run/'paired_comparisons.csv',index=False)
for kind in ('soc','first_ms'):
 fig,axes=plt.subplots(2,4,figsize=(16,7),sharex=True)
 for ax,mapname in zip(axes.flat,maps):
  for method,column in (('lacam_dfs','lacam_'+kind),('ir','ir_'+kind)):
   subset=pairs[pairs['map']==mapname].groupby('num_agents')[column].median()
   ax.plot(subset.index,subset.values,'o-',label=labels[method],color=colors[method])
  ax.set(title=mapname,xticks=[250,300,350,400,450,500],xlabel='Number of agents',ylabel='Median SOC' if kind=='soc' else 'Median first solution (ms)')
  ax.grid(alpha=.25)
 axes[0,0].legend(); fig.suptitle(f'Paired successful instances only; {len(pairs)} pairs'); fig.tight_layout()
 fig.savefig(run/f'group_{kind}.png',dpi=180); fig.savefig(run/f'group_{kind}.pdf'); plt.close(fig)
size_summary=df.groupby(['num_agents','method']).agg(completed=('valid_solution','size'),solved=('valid_solution','sum')).reset_index()
size_summary['success_rate']=100*size_summary.solved/size_summary.completed
size_summary.to_csv(run/'size_summary.csv',index=False)
summary=df.groupby('method').agg(completed=('valid_solution','size'),solved=('valid_solution','sum'))
summary['success_rate']=100*summary.solved/summary.completed
failures=df[df.valid_solution==0][['case_id','method','num_agents','exit_code','wall_time_s']].copy()
failures['outcome']=['基础设施异常' if r.get('infrastructure_error') else '搜索超时（solver 自报）' if r.get('timed_out') else '未解出（日志未给出超时标记）' for r in rows if not r['valid_solution']]
failures['trace']=['<a href="artifacts/'+r.case_id+'/'+r.method+'/command.json">command</a> · <a href="artifacts/'+r.case_id+'/'+r.method+'/stdout.log.gz">stdout</a> · <a href="artifacts/'+r.case_id+'/'+r.method+'/result.json">result</a>' for r in failures.itertuples()]
html='''<!doctype html><html lang="zh"><meta charset="utf-8"><title>Group 250–500 对比</title><style>body{font:16px system-ui;max-width:1400px;margin:40px auto;padding:20px}img{width:100%}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #ddd}input{padding:8px;width:400px}</style><h1>Group 250–500 agents</h1><p>仓库原生生成器；group size=5（连通分量的末组可能小于 5）；8 地图 × 6 规模 × 20 测例；外部初始随机种子 1。两个方法均使用 10 秒搜索预算。每次运行绑定一个 CPU，保留 CPU 124–127。250–500 属于扩展实验，没有对应论文数值，不能视为论文复现点。</p>'''
if provenance.get('ir_variant')=='distance_cache_fixed':
 html+='<p><strong>IR 使用缓存修复版。LaCAM 结果来自同一批输入的原版实验，无重复运行。</strong>修复仅将 BFS 节点展开移到查询返回之前。<a href="../comparison.html">查看原版 IR 统计</a> · <a href="../ir_diagnosis/distance_cache_fix.patch">修复补丁</a></p>'
else:
 html+='<p><strong>此页是原版 IR 统计，其中包含已确认的距离缓存错误。</strong>不能把所有“No solution found”解释为任务无解或搜索超时。<a href="ir_diagnosis/cache_replay.html">逐步复现缓存错误</a> · <a href="ir_diagnosis/early_failure_audit.json">真实失败测例追踪</a> · <a href="cache_fixed/comparison.html">修复版全量对照（完成后可用）</a></p>'
html+='<h2>核查</h2><pre>'+json.dumps(audit,ensure_ascii=False,indent=2)+'</pre><h2>成功率</h2>'+summary.to_html()+'<img src="group_success.png"><p><a href="group_success.pdf">PDF 图</a> · <a href="rows.csv">全部结果 CSV</a> · <a href="success_rates.csv">分规模统计</a></p><h2>各规模成功率</h2>'+size_summary.to_html(index=False)+'<h2>双方都成功的同一批测例：成本与首解时间</h2><p>这里仅比较双方均成功的 '+str(len(pairs))+' 个测例。按地图、规模取中位数；未解出不填成 0。</p><img src="group_soc.png"><img src="group_first_ms.png"><p><a href="paired_comparisons.csv">逐测例配对统计</a> · <a href="group_soc.pdf">SOC PDF</a> · <a href="group_first_ms.pdf">首解时间 PDF</a></p><h2>未解出测例与真实日志</h2><input id="filter" placeholder="输入地图、方法或规模筛选">'+failures.to_html(index=False,escape=False,table_id='failures')
html+='''<script>document.querySelector('#filter').oninput=e=>{for(const row of document.querySelectorAll('#failures tbody tr'))row.hidden=!row.textContent.toLowerCase().includes(e.target.value.toLowerCase())}</script></html>'''
(run/'comparison.html').write_text(html)
status=json.loads((run/'status.json').read_text()); status.update(state='complete' if audit['complete'] else 'needs_review',audit_complete=audit['complete']); atomic_json(run/'status.json',status)
atomic_json(run/'summary_final.json',summary.reset_index().to_dict(orient='records'))
print(json.dumps(audit),flush=True)
if not audit['complete']: raise SystemExit(1)
