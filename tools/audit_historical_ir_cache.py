#!/usr/bin/env python3
"""Trace every historical IR failure, then replay that subset with the cache fix."""
import os,json,subprocess,concurrent.futures,hashlib,time,sys
from pathlib import Path
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
repo=Path(__file__).resolve().parents[1];run=repo/'build/results/paper_2609_14208_repository';out=run/'cache_bug_review';out.mkdir(exist_ok=True)
manifest_path=repo/'data/paper_2609_14208_repository/manifest.json';manifest=json.loads(manifest_path.read_text())
rows=[json.loads(p.read_text()) for p in (run/'artifacts').glob('*/*/ir/result.json')];failed=[r for r in rows if not r['valid_solution']];assert len(rows)==9760 and len(failed)==418
ids={r['case_id'] for r in failed};subset={**manifest,'cases':[c for c in manifest['cases'] if c['case_id'] in ids],'review_scope':'Only the 418 previously unsuccessful IR records; not a full corrected benchmark','source_manifest_sha256':hashlib.sha256(manifest_path.read_bytes()).hexdigest()};assert len(subset['cases'])==418
subset_path=out/'failed_manifest.json';subset_path.write_text(json.dumps(subset,indent=2))
trace_binary=repo/'build/results/group_250_500_repository/ir_diagnosis/ir_tapf_trace';binary_sha=hashlib.sha256(trace_binary.read_bytes()).hexdigest()
def trace(item):
 index,row=item;folder=out/'original_trace'/row['case_id'];folder.mkdir(parents=True,exist_ok=True)
 command=['taskset','-c',str(index%56),str(trace_binary),'solve','--matrix',row['matrix_file'],'--solver','dbs_hungarian','--max-iterations','100000','--time-limit-sec','10']
 env={**os.environ,'RAYON_NUM_THREADS':'1'};env.pop('TAPF_FIX_DISTANCE_CACHE',None);t=time.monotonic()
 with (folder/'stdout.log').open('wb') as stdout,(folder/'stderr.log').open('wb') as stderr:
  p=subprocess.run(command,stdout=stdout,stderr=stderr,timeout=130,env=env)
 text=(folder/'stderr.log').read_text();stdout=(folder/'stdout.log').read_text()
 result={'case_id':row['case_id'],'suite':row['suite'],'map':row['map'],'num_agents':row['num_agents'],'seed':row['seed'],'argv':command,'exit_code':p.returncode,'fixture_sha256':row['fixture_sha256'],'trace_binary_sha256':binary_sha,'wall_time_s':time.monotonic()-t,'original_wall_time_s':row['wall_time_s'],
 'false_unreachable_queries':text.count('CACHE_FALSE_UNREACHABLE'),'assignment_failed':'INITIAL_ASSIGNMENT_FAILED' in text,'initial_stage_failed':'STAGE initial_assignment' in text and 'STAGE fixed_goal_path_search' not in text,'initial_path_failed':'INITIAL_PATH_FAILED' in text,'no_solution':'No solution found' in stdout}
 (folder/'trace.json').write_text(json.dumps(result,indent=2));return result
results=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=56) as pool:
 futures=[pool.submit(trace,item) for item in enumerate(failed)]
 for future in concurrent.futures.as_completed(futures):
  results.append(future.result())
  if len(results)%50==0:print('historical failure traces',len(results),'/',len(failed),flush=True)
summary={'historical_ir_runs':9760,'previous_ir_failures':418,'traced':len(results),'by_suite':{},'records':results}
for suite in ('group','common'):
 group=[r for r in results if r['suite']==suite];summary['by_suite'][suite]={'failures':len(group),'with_confirmed_cache_false_unreachable':sum(r['false_unreachable_queries']>0 for r in group),'initial_stage_failed':sum(r['initial_stage_failed'] for r in group),'initial_path_failed':sum(r['initial_path_failed'] for r in group)}
(out/'trace_summary.json').write_text(json.dumps(summary,indent=2));print({k:v for k,v in summary.items() if k!='records'},flush=True)
fixed=repo/'build/results/group_250_500_repository/ir_diagnosis/ir_tapf_cache_fixed'
command=[sys.executable,str(repo/'tools/run_paper_2609_14208.py'),'--manifest',str(subset_path),'--out-dir',str(out/'fixed_replays'),'--methods','ir','--ir-binary',str(fixed),'--jobs','56','--validation-jobs','8','--skip-report']
with (out/'fixed_run.log').open('wb') as log:subprocess.run(command,stdout=log,stderr=log,check=True)
print('FIXED REPLAYS FINISHED',flush=True)
