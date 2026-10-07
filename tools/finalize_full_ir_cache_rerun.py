#!/usr/bin/env python3
"""Audit a complete fixed IR rerun, retain other methods, and publish comparisons."""
import os
os.sched_setaffinity(0, set(os.sched_getaffinity(0)) - {124, 125, 126, 127})
for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):os.environ[name]='1'
import collections,csv,hashlib,html,json,subprocess,sys,time
from pathlib import Path
repo=Path(__file__).resolve().parents[1];base=repo/'build/results';old=base/'paper_2609_14208_repository';fixed=base/'paper_2609_14208_cache_fixed';out=base/'paper_2609_14208_cache_comparison';manifest=repo/'data/paper_2609_14208_repository/manifest.json'
def read(p):return json.loads(p.read_text())
def write(p,value):p.write_text(json.dumps(value,ensure_ascii=False,indent=2))
def link(p,target):
 p.parent.mkdir(parents=True,exist_ok=True)
 if p.is_symlink() or p.exists():assert p.resolve()==target.resolve(),p
 else:p.symlink_to(target.resolve(),target_is_directory=target.is_dir())
# Wait only for this exact live runner; a PID alone does not identify the job.
pid=int((fixed/'run.pid').read_text())
while True:
 try:cmd=Path(f'/proc/{pid}/cmdline').read_bytes()
 except FileNotFoundError:break
 if b'run_paper_2609_14208.py' not in cmd or fixed.name.encode() not in cmd:break
 time.sleep(5)
cases=read(manifest)['cases'];rows=[read(p) for p in (fixed/'artifacts').glob('*/*/*/result.json')]
assert len(rows)==len(cases)==9760
assert {(r['case_id'],r['method']) for r in rows}=={(c['case_id'],'ir') for c in cases}
assert not any('infrastructure_error' in r for r in rows)
fprov=read(fixed/'provenance.json');oprov=read(old/'provenance.json')
assert fprov['dataset_manifest_sha256']==hashlib.sha256(manifest.read_bytes()).hexdigest()
assert fprov['jobs']==112 and len(fprov['reserved_cpus'])>=4
out.mkdir(exist_ok=True)
by_case = {r['case_id']: r for r in rows}
for c in cases:
 r = by_case[c['case_id']]
 for key in ('fixture_sha256', 'matrix_sha256', 'map_sha256', 'seed', 'num_agents', 'suite', 'scenario'):
  assert r[key] == c[key], (c['case_id'], key)
 assert r['time_limit'] == 10 and r['binary_sha256'] == fprov['binaries_sha256']['ir']
 for kind in ('fixture', 'matrix'):
  assert hashlib.sha256(Path(c[kind+'_file']).read_bytes()).hexdigest() == c[kind+'_sha256'], c['case_id']
write(out/'ir_input_audit.json', {'complete':True,'cases_checked':len(cases),
 'checks':['case coverage','fixture hash','matrix hash','map hash recorded','seed','agent count','suite','scenario','10 s budget','fixed binary hash']})
for c in cases:
 for method,source in [('ir',fixed),('lacam_dfs',old),('itacbs',old)]:
  target=source/'artifacts'/c['case_id']/method
  assert (target/'result.json').exists()
  link(out/'artifacts'/c['case_id']/method,target)
  cached=source/'archive_audit_cache'/c['case_id']/method
  if cached.exists():link(out/'archive_audit_cache'/c['case_id']/method,cached)
prov={**fprov,'methods':['lacam_dfs','ir','itacbs'],'source_runs':{'ir':str(fixed),'lacam_dfs':str(old),'itacbs':str(old)},'ir_variant':'distance_cache_fixed','binaries_sha256':{**oprov['binaries_sha256'],'ir':fprov['binaries_sha256']['ir']}}
write(out/'provenance.json',prov);write(out/'status.json',{'state':'auditing','completed':29280,'total':29280})
from run_paper_2609_14208 import export
allrows=[read(p) for p in (out/'artifacts').glob('*/*/*/result.json')];export(out,allrows)
subprocess.run([sys.executable,str(repo/'tools/report_paper_2609_14208.py'),'--run-dir',str(out),'--manifest',str(manifest),'--audit-jobs','32'],check=True)
audit=read(out/'coverage_audit.json');assert audit['complete'],audit
fixed_success=sum(r['valid_solution'] for r in rows)
write(fixed/'coverage_audit.json',{'complete':True,'expected_rows':9760,'actual_rows':len(rows),'archived_solutions_checked':fixed_success,'combined_audit':str(out/'coverage_audit.json'),'infrastructure_errors':0})
previous={read(p)['case_id']:read(p) for p in (old/'artifacts').glob('*/*/ir/result.json')};assert len(previous)==9760
changes=[{'case_id':r['case_id'],'suite':r['suite'],'original_solved':previous[r['case_id']]['valid_solution'],'fixed_solved':r['valid_solution'],'original_soc':previous[r['case_id']].get('soc'),'fixed_soc':r.get('soc')} for r in rows]
with (out/'ir_before_after_cases.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(changes[0]));w.writeheader();w.writerows(changes)
trans={suite:dict(collections.Counter(str((r['original_solved'],r['fixed_solved'])) for r in changes if r['suite']==suite)) for suite in ['group','common']};write(out/'cache_fix_effect.json',trans)
link(out/'paper.pdf',old/'paper.pdf')
subprocess.run([sys.executable,str(repo/'tools/plot_paper_2609_comparison.py'),'--run-dir',str(out),'--cache-fixed'],check=True)
# Draw all actually measured Group points, including the completed extension.
import pandas as pd,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
cols=['suite','method','map','num_agents','valid_solution'];original=pd.read_csv(old/'rows.csv',usecols=cols);extension=pd.read_csv(base/'group_250_500_repository/rows.csv',usecols=cols)
raw=pd.concat([original,extension]);raw=raw[(raw.suite=='group')&raw.method.isin(['lacam_dfs','ir'])].copy();raw['version']=raw.method.map({'lacam_dfs':'ITA-LaCAM','ir':'IR original cache'})
f=pd.concat([pd.read_csv(fixed/'rows.csv',usecols=cols),pd.read_csv(base/'group_250_500_cache_fixed/rows.csv',usecols=cols)]);f=f[f.suite=='group'].copy();f['version']='IR cache fixed'
metrics=pd.concat([raw,f]).groupby(['map','num_agents','version']).agg(completed=('valid_solution','size'),solved=('valid_solution','sum')).reset_index();assert all(metrics.completed==20);metrics['success_rate']=100*metrics.solved/metrics.completed;metrics.to_csv(out/'group_10_500_success_rates.csv',index=False)
fig,axes=plt.subplots(2,4,figsize=(16,7),sharex=True,sharey=True);maps=list(dict.fromkeys(c['map'] for c in cases if c['suite']=='group'))
for ax,mapname in zip(axes.flat,maps):
 ax.axvspan(225,510,color='#edf3fa')
 for version,color,style in [('ITA-LaCAM','#d62728','-'),('IR original cache','#2ca02c','--'),('IR cache fixed','#1f77b4','-')]:
  g=metrics[(metrics['map']==mapname)&(metrics.version==version)].sort_values('num_agents');ax.plot(g.num_agents,g.success_rate,style,color=color,label=version)
 ax.set(title=mapname,xlim=(0,510),ylim=(-3,103),xlabel='Agents',ylabel='Success (%)');ax.grid(alpha=.2)
axes[0,0].legend(fontsize=8);fig.tight_layout()
for ext in ['png','pdf']:fig.savefig(out/f'group_10_500_success.{ext}',dpi=180)
plt.close(fig)
summary=read(out/'summary.json');before=read(old/'summary.json');table=''
for suite in ['group','common']:
 for method,label in [('lacam_dfs','ITA-LaCAM（原结果）'),('ir','IR（全量缓存修复版）'),('itacbs','ITA-CBS（原结果）')]:
  r=next(x for x in summary if x['suite']==suite and x['method']==method);o=next(x for x in before if x['suite']==suite and x['method']==method)
  table+=f'<tr><td>{suite}</td><td>{label}</td><td>{o["solved"]}/{o["completed"]} ({o["success_rate"]*100:.2f}%)</td><td>{r["solved"]}/{r["completed"]} ({r["success_rate"]*100:.2f}%)</td></tr>'
sections=''.join(f'<h2>论文图 {n}</h2><img src="paper_comparison/comparison_figure{n}.png"><p><a href="paper_comparison/rerun_figure{n}.pdf">修复版数据图 PDF</a></p>' for n in range(2,7))
page='<!doctype html><meta charset="utf-8"><title>全量 IR 缓存修复对比</title><style>body{font:16px system-ui;max-width:1450px;margin:30px auto;padding:20px;color:#183047;background:#f5f7fa}img{width:100%}td,th{padding:12px;border-bottom:1px solid #ddd;text-align:left}table{width:100%;border-collapse:collapse}p{line-height:1.8}</style><h1>IR 缓存修复：9,760 个实例全量重跑</h1><p>相同输入与 seed；每例 10 秒搜索预算。求解并行度由 56 提高到 112，8 路验证，预留 4 个逻辑 CPU。IR 每例只使用此次修复版运行结果，不与原成功取最佳值。LaCAM 与 CBS 复用旧版已验证结果，其中保留此前授权的超时重放。</p><p><a href="../comparison.html">返回原对比页</a> · <a href="coverage_audit.json">全量归档轨迹复核</a> · <a href="ir_before_after_cases.csv">IR 逐例修复前后 CSV</a> · <a href="cache_fix_effect.json">成功与失败转换统计</a> · <a href="rows.csv">完整对比数据</a></p><table><tr><th>实验</th><th>方法</th><th>原统计</th><th>修复版对比</th></tr>'+table+'</table><h2>Group 10–500：完整修复版曲线</h2><img src="group_10_500_success.png"><p>10–200 来自此次全量重跑，250–500 来自此前完成的修复版扩展实验，每点 20 个测例。</p>'+sections
(out/'comparison.html').write_text(page)
link(old/'full_ir_cache_fixed',out);link(old/'full_ir_cache_fixed_runs',fixed)
write(out/'completion_audit.json',{'complete':True,'ir_rerun_cases':9760,'ir_success_archives_checked':fixed_success,'combined_records':len(allrows),'combined_archive_audit':audit,'published_at':time.strftime('%Y-%m-%dT%H:%M:%S%z')})
subprocess.run([sys.executable,str(repo/'tools/build_paper_comparison_page.py'),'--run-dir',str(old)],check=True)
print('FULL FIXED IR RERUN FINISHED',summary,trans,flush=True)
