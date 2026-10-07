#!/usr/bin/env python3
"""Run isolated, validated ITA ablations on identical cases without merging incumbents."""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import argparse,concurrent.futures,gzip,hashlib,json,queue,random,subprocess,sys,time,threading
from pathlib import Path
from run_full_three_method_experiment import parse_kv
from run_paper_2609_14208 import memory_available_gib
repo=Path(__file__).resolve().parents[1];root=repo/'build/results/ita_optimization_500'
p=argparse.ArgumentParser();p.add_argument('--cases',type=Path,default=root/'screen_cases.json');p.add_argument('--variants',nargs='+',default=['baseline','lazy_metrics','revisit','both','fast_checks']);p.add_argument('--trace',action='store_true');p.add_argument('--mode',choices=['dfs','focal'],default='dfs');p.add_argument('--jobs',type=int,default=56);p.add_argument('--name',default='screen');p.add_argument('--time-limit',type=float,default=10);a=p.parse_args();os.sched_setaffinity(0,set(range(124)))
items=json.loads(a.cases.read_text());out=root/a.name;out.mkdir(exist_ok=True);cpus=queue.Queue()
for i in range(a.jobs):cpus.put(i)
binaries={v:root/'variants'/v/'tapf_benchmark' for v in a.variants};hashes={v:hashlib.sha256(b.read_bytes()).hexdigest() for v,b in binaries.items()}
tasks=[(v,pair) for pair in items for v in a.variants];random.Random(88117).shuffle(tasks)
(out/'provenance.json').write_text(json.dumps({'jobs':a.jobs,'search_mode':a.mode,'reserved_cpus':[124,125,126,127],'time_limit':a.time_limit,'binary_sha256':hashes,'case_file':str(a.cases),'case_file_sha256':hashlib.sha256(a.cases.read_bytes()).hexdigest(),'started_at':time.strftime('%Y-%m-%dT%H:%M:%S%z')},indent=2));(out/'run.pid').write_text(str(os.getpid()));start=time.monotonic()
def work(task):
 variant,pair=task;row=pair['lacam'];folder=out/variant/row['case_id'];folder.mkdir(parents=True,exist_ok=True);record=folder/'result.json'
 if record.exists():
  old=json.loads(record.read_text());assert old['binary_sha256']==hashes[variant] and old['fixture_sha256']==row['fixture_sha256'];return old
 cpu=cpus.get();schedule=folder/'schedule.yaml';argv=['taskset','-c',str(cpu),str(binaries[variant]),row['fixture_file'],'',str(a.time_limit),str(schedule),'1','0',str(row['seed']),a.mode];began=time.monotonic()
 r={'case_id':row['case_id'],'variant':variant,'suite':row['suite'],'map':row['map'],'num_agents':int(row['num_agents']),'seed':int(row['seed']),'fixture_file':row['fixture_file'],'fixture_sha256':row['fixture_sha256'],'binary_sha256':hashes[variant],'original_lacam_success':bool(int(row['valid_solution'])),'fixed_ir_success':bool(int(pair['fixed_ir']['valid_solution'])),'any_ir_counterexample':pair['any_ir_counterexample'],'argv':argv,'time_limit':a.time_limit,'valid_solution_verified':False}
 try:
  while memory_available_gib()<32:time.sleep(1)
  with (folder/'stdout.log').open('wb') as stdout,(folder/'stderr.log').open('wb') as stderr:proc=subprocess.run(argv,stdout=stdout,stderr=stderr,timeout=a.time_limit+120,env=dict(os.environ, **({'TAPF_SEARCH_TRACE':str(folder/'search.jsonl')} if a.trace else {})))
  r['exit_code']=proc.returncode;r.update(parse_kv((folder/'stdout.log').read_text()));assert proc.returncode==0 or (proc.returncode==1 and r.get('valid_instance')==1 and r.get('solved')==0),proc.returncode
  if r.get('solved'):
   command=['taskset','-c',str(cpu),sys.executable,str(repo/'tools/recover_validation_pool.py'),'validate','--fixture',row['fixture_file'],'--schedule',str(schedule)]
   validation=subprocess.run(command,capture_output=True,text=True,timeout=600);(folder/'validation.json').write_text(json.dumps({'argv':command,'exit_code':validation.returncode,'stdout':validation.stdout,'stderr':validation.stderr},indent=2));assert validation.returncode==0,validation.stderr
   metrics=json.loads(validation.stdout);assert metrics['soc']==r['soc'],(metrics,r);assert r['first_solution_time_ms']<=a.time_limit*1000;r['valid_solution_verified']=True;r['verified_metrics']=metrics
  for path in [schedule,Path(str(schedule)+'.bin')]:
   if path.exists():
    with gzip.open(str(path)+'.gz','wb') as f:f.write(path.read_bytes())
    path.unlink()
 except Exception as e:r['infrastructure_error']=repr(e)
 finally:cpus.put(cpu)
 r['wall_time_s']=time.monotonic()-began;record.write_text(json.dumps(r,indent=2));return r
rows=[];last=0
with concurrent.futures.ThreadPoolExecutor(max_workers=a.jobs) as pool:
 futures=[pool.submit(work,t) for t in tasks]
 for future in concurrent.futures.as_completed(futures):
  rows.append(future.result())
  if time.monotonic()-last>5 or len(rows)==len(tasks):
   status={'pid':os.getpid(),'completed':len(rows),'total':len(tasks),'elapsed_sec':time.monotonic()-start,'errors':sum('infrastructure_error' in r for r in rows),'state':'complete' if len(rows)==len(tasks) else 'running'};(out/'status.json').write_text(json.dumps(status,indent=2));print(status,flush=True);last=time.monotonic()
(out/'results.json').write_text(json.dumps(rows,indent=2));assert not any('infrastructure_error' in r for r in rows)
summary={}
for variant in a.variants:
 rs=[r for r in rows if r['variant']==variant];summary[variant]={name:{'cases':len(group),'solved':sum(r['valid_solution_verified'] for r in group)} for name,group in [('all',rs),('counterexamples',[r for r in rs if r['any_ir_counterexample']]),('original_failures',[r for r in rs if not r['original_lacam_success']]),('success_controls',[r for r in rs if r['original_lacam_success']])]}
(out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
