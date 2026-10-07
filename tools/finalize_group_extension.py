#!/usr/bin/env python3
"""Combine independently recorded Group runs, audit archives and publish local plots."""
import argparse, collections, hashlib, json, os, subprocess, sys, time
from pathlib import Path
os.environ['OPENBLAS_NUM_THREADS']='1'
p=argparse.ArgumentParser();p.add_argument('--original',type=Path,required=True);p.add_argument('--fixed',type=Path,required=True);p.add_argument('--out-dir',type=Path,required=True);p.add_argument('--manifest',type=Path,required=True)
a=p.parse_args();original=a.original.resolve();fixed=a.fixed.resolve();out=a.out_dir.resolve();manifest=a.manifest.resolve()
out.mkdir(parents=True,exist_ok=True)
def atomic(path,data):
 temp=path.with_suffix(path.suffix+'.tmp');temp.write_text(json.dumps(data,indent=2));temp.replace(path)
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
pid=int((fixed/'run.pid').read_text())
while Path(f'/proc/{pid}/cmdline').exists():
 try:cmd=Path(f'/proc/{pid}/cmdline').read_bytes()
 except OSError:break
 if b'run_paper_2609_14208.py' not in cmd or str(fixed.name).encode() not in cmd:break
 time.sleep(5)
status=json.loads((fixed/'status.json').read_text());cases=json.loads(manifest.read_text())['cases']
fixed_rows=[json.loads(path.read_text()) for path in (fixed/'artifacts').glob('*/*/*/result.json')]
assert len(fixed_rows)==960 and status['completed']==960,status
assert {(r['case_id'],r['method']) for r in fixed_rows}=={(c['case_id'],'ir') for c in cases}
assert not any(r.get('infrastructure_error') for r in fixed_rows),'IR failures require investigation'
fixed_provenance=json.loads((fixed/'provenance.json').read_text());old_provenance=json.loads((original/'provenance.json').read_text())
for c in cases:
 parent=out/'artifacts'/c['case_id'];parent.mkdir(parents=True,exist_ok=True)
 for method,source in (('lacam_dfs',original),('ir',fixed)):
  target=source/'artifacts'/c['case_id']/method
  assert (target/'result.json').exists(),target
  link=parent/method
  if not link.exists():link.symlink_to(target,target_is_directory=True)
  else:assert link.resolve()==target
provenance={**fixed_provenance,'methods':['lacam_dfs','ir'],'ir_variant':'distance_cache_fixed',
 'binaries_sha256':{'lacam_dfs':old_provenance['binaries_sha256']['lacam_dfs'],'ir':fixed_provenance['binaries_sha256']['ir']},
 'source_runs':{'lacam_dfs':str(original),'ir':str(fixed)},'reused_lacam_results':True,
 'original_ir_binary_sha256':old_provenance['binaries_sha256']['ir'],
 'cache_fix_patch_sha256':sha(original/'ir_diagnosis/distance_cache_fix.patch')}
atomic(out/'provenance.json',provenance)
atomic(out/'status.json',{'state':'solvers_finished','completed':1920,'total':1920,'reserved_cpus':provenance['reserved_cpus']})
subprocess.run([sys.executable,str(Path(__file__).with_name('report_group_extension.py')),'--run-dir',str(out),'--manifest',str(manifest)],check=True)
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
raw=[json.loads(path.read_text()) for path in (original/'artifacts').glob('*/*/*/result.json')]
old_ir={r['case_id']:r for r in raw if r['method']=='ir'}
la={r['case_id']:r for r in raw if r['method']=='lacam_dfs'}
new_ir={r['case_id']:r for r in fixed_rows}
versions={'ITA-LaCAM':la,'IR-TAPF (original cache)':old_ir,'IR-TAPF (cache fixed)':new_ir}
records=[{**r,'version':version} for version,items in versions.items() for r in items.values()]
df=pd.DataFrame(records)
summary=df.groupby('version').agg(completed=('valid_solution','size'),solved=('valid_solution','sum')).reset_index();summary['success_rate']=100*summary.solved/summary.completed
atomic(out/'before_after_summary.json',summary.to_dict(orient='records'))
metrics=df.groupby(['map','num_agents','version']).agg(completed=('valid_solution','size'),solved=('valid_solution','sum')).reset_index();metrics['success_rate']=100*metrics.solved/metrics.completed;metrics.to_csv(out/'before_after_success_rates.csv',index=False)
changes=[]
for c in cases:
 o=old_ir[c['case_id']];n=new_ir[c['case_id']]
 changes.append({'case_id':c['case_id'],'map':c['map'],'num_agents':c['num_agents'],'seed':c['seed'],
  'original_solved':o['valid_solution'],'fixed_solved':n['valid_solution'],
  'original_soc':o.get('soc') if o['valid_solution'] else None,'fixed_soc':n.get('soc') if n['valid_solution'] else None,
  'original_first_ms':o.get('initial_solution_time_ms') if o['valid_solution'] else None,
  'fixed_first_ms':n.get('initial_solution_time_ms') if n['valid_solution'] else None})
changes=pd.DataFrame(changes);changes.to_csv(out/'ir_before_after_cases.csv',index=False)
transition=collections.Counter((int(r.original_solved),int(r.fixed_solved)) for r in changes.itertuples())
audit=json.loads((out/'coverage_audit.json').read_text());assert audit['complete'],audit
fix_audit={**audit,'expected_rows':960,'actual_rows':960,'archived_solutions_checked':sum(r['valid_solution'] for r in fixed_rows),'scope':'IR cache-fixed solver run; complete archive audit performed by combined report','combined_audit_file':str(out/'coverage_audit.json')}
atomic(fixed/'coverage_audit.json',fix_audit);status.update(state='complete',audit_complete=True);atomic(fixed/'status.json',status)
fig,axes=plt.subplots(2,4,figsize=(16,7),sharex=True,sharey=True)
colors=['#d62728','#2ca02c','#1f77b4']
for ax,name in zip(axes.flat,dict.fromkeys(c['map'] for c in cases)):
 for version,color in zip(versions,colors):
  s=metrics[(metrics['map']==name)&(metrics.version==version)]
  ax.plot(s.num_agents,s.success_rate,'o--' if version=='IR-TAPF (original cache)' else 'o-',color=color,label=version)
 ax.set(title=name,ylim=(-3,103),xticks=[250,300,350,400,450,500],xlabel='Number of agents',ylabel='Success rate (%)');ax.grid(alpha=.25)
axes[0,0].legend(fontsize=8);fig.suptitle('Group 250–500: identical instances, 20 cases per size, 10 s search budget');fig.tight_layout();fig.savefig(out/'group_success_before_after.png',dpi=180);fig.savefig(out/'group_success_before_after.pdf');plt.close(fig)
diagnosis=json.loads((original/'ir_diagnosis/early_failure_audit.json').read_text())
recovered_cache_cases=sum(new_ir[r['case_id']]['valid_solution'] for r in diagnosis['results'])
report={'total_instances':960,'ir_original_solved':sum(r['valid_solution'] for r in old_ir.values()),'ir_fixed_solved':sum(r['valid_solution'] for r in new_ir.values()),
 'original_failed_fixed_passed':transition[(0,1)],'original_passed_fixed_failed':transition[(1,0)],
 'both_passed':transition[(1,1)],'both_failed':transition[(0,0)],
 'confirmed_early_cache_failures':diagnosis['early_failures_checked'],'confirmed_early_cache_failures_recovered':recovered_cache_cases}
atomic(out/'cache_fix_effect.json',report)
page=out/'comparison.html';html=page.read_text();addition='<h2>修复前后：同一批输入</h2>'+summary.to_html(index=False)+'<p>原版失败→修复版成功：'+str(transition[(0,1)])+'；原版成功→修复版失败：'+str(transition[(1,0)])+'。两组各只运行一次，未取多次尝试的最佳值。单次运行差异也可能包含搜索随机性与计时波动。</p><img src="group_success_before_after.png"><p><a href="group_success_before_after.pdf">PDF 对比图</a> · <a href="ir_before_after_cases.csv">逐测例修复前后对照</a> · <a href="cache_fix_effect.json">缓存修复影响统计</a> · <a href="../ir_diagnosis/cache_replay.html">缓存 bug 逐步回放</a></p>'
html=html.replace('<h2>核查</h2>',addition+'<h2>核查</h2>',1);page.write_text(html)
primary=original/'comparison.html';html=primary.read_text();html=html.replace('<h2>核查</h2>','<p><strong>修复版全量对照已完成。</strong><a href="cache_fixed/comparison.html">查看修复前后图与逐测例变化</a></p><h2>核查</h2>',1);primary.write_text(html)
# Keep export sources beside data; original runs remain authoritative.
source=out/'sources';source.mkdir(exist_ok=True)
for name in ('finalize_group_extension.py','report_group_extension.py','recover_validation_pool.py','run_paper_2609_14208.py'):
 (source/name).write_bytes(Path(__file__).with_name(name).read_bytes())
atomic(out/'delivery.json',{'state':'complete','summary':report,'input_manifest':str(manifest),'original_run':str(original),'cache_fixed_run':str(fixed),'comparison_url':'http://127.0.0.1:8766/cache_fixed/comparison.html'})
print(json.dumps(report),flush=True)
