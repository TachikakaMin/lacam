#!/usr/bin/env python3
"""Controlled diagnostics; never overwrite benchmark results."""
import argparse,collections,concurrent.futures,gzip,hashlib,json,os,subprocess,sys,time
from pathlib import Path
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
import yaml
from run_full_three_method_experiment import parse_kv
from run_ir_lacam_cross_experiment import load_map
repo=Path(__file__).resolve().parents[1];root=repo/'build/results/group_250_500_cache_comparison/lacam_failure_analysis'
p=argparse.ArgumentParser();p.add_argument('--mode',choices=['replay','controls'],required=True);p.add_argument('--case-ids',nargs='*');p.add_argument('--control-variants',nargs='+',default=['cpp_initial_fixed','ir_initial_fixed']);p.add_argument('--binary',type=Path);p.add_argument('--variant-prefix',default='');a=p.parse_args()
cases=json.loads((root.parent/'ir_success_lacam_failure.json').read_text())
if a.case_ids:cases=[c for c in cases if c['lacam']['case_id'] in a.case_ids]
binary=a.binary.resolve() if a.binary else repo/'build/tapf_benchmark';ir_binary=repo/'build/ir_diagnostics_original/target/release/ir_tapf'

def solve(row,variant,fixture,cpu):
 variant=a.variant_prefix+variant
 folder=root/row['case_id']/variant;folder.mkdir(parents=True,exist_ok=True)
 schedule=folder/'schedule.yaml';argv=['taskset','-c',str(cpu),str(binary),str(fixture),'','10',str(schedule),'1','0',str(row['seed']),'dfs']
 t=time.monotonic()
 with (folder/'stdout.log').open('wb') as out,(folder/'stderr.log').open('wb') as err:
  process=subprocess.run(argv,stdout=out,stderr=err,timeout=130,env={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'})
 stats=parse_kv((folder/'stdout.log').read_text());result={'case_id':row['case_id'],'variant':variant,'argv':argv,'exit_code':process.returncode,'original_fixture_sha256':row['fixture_sha256'],'diagnostic_fixture_sha256':hashlib.sha256(fixture.read_bytes()).hexdigest(),'wall_time_s':time.monotonic()-t,**stats,'valid_solution_verified':False}
 if result.get('solved'):
  # Check against the ORIGINAL unrestricted task, not only the restricted control.
  command=['taskset','-c',str(cpu),sys.executable,str(repo/'tools/recover_validation_pool.py'),'validate','--fixture',row['fixture_file'],'--schedule',str(schedule)]
  v=subprocess.run(command,capture_output=True,text=True,timeout=600,env={**os.environ,'OPENBLAS_NUM_THREADS':'1'})
  (folder/'independent_validation.json').write_text(json.dumps({'argv':command,'exit_code':v.returncode,'stdout':v.stdout,'stderr':v.stderr},indent=2))
  assert v.returncode==0,v.stderr
  result['valid_solution_verified']=result['first_solution_time_ms']<=10000
 for path in (schedule,Path(str(schedule)+'.bin')):
  if path.exists():
   with gzip.open(str(path)+'.gz','wb') as f:f.write(path.read_bytes())
   path.unlink()
 (folder/'result.json').write_text(json.dumps(result,indent=2));return result

def work(item):
 index,pair=item;row=pair['lacam'];cpu=index%8;folder=root/row['case_id'];folder.mkdir(parents=True,exist_ok=True)
 if a.mode=='replay':return [solve(row,'original_replay',Path(row['fixture_file']),cpu)]
 fixture=Path(row['fixture_file']);data=yaml.safe_load(fixture.read_text());map_path=fixture.parent/data['map'];mapping=load_map(map_path)
 cpp=subprocess.run(['taskset','-c',str(cpu),str(root/'tapf_initial_assignment'),str(fixture)],capture_output=True,text=True,check=True)
 (folder/'cpp_initial_assignment.json').write_text(cpp.stdout);cpp_goals=json.loads(cpp.stdout)['goals']
 output=folder/'ir_initial_goal_ids.json';command=['taskset','-c',str(cpu),str(ir_binary),'solve','--matrix',row['matrix_file'],'--solver','dbs_hungarian','--max-iterations','100000','--time-limit-sec','10']
 ir=subprocess.run(command,capture_output=True,text=True,timeout=130,env={**os.environ,'TAPF_INITIAL_GOALS_OUTPUT':str(output),'TAPF_ASSIGNMENT_ONLY':'1','RAYON_NUM_THREADS':'1'})
 (folder/'ir_assignment_only.json').write_text(json.dumps({'argv':command,'exit_code':ir.returncode,'stdout':ir.stdout,'stderr':ir.stderr,'purpose':'dump initial goals without running MAPF; No solution found here is intentional'},indent=2));assert ir.returncode==0 and output.exists()
 ids=json.loads(output.read_text());inverse={v:k for k,v in mapping.coord_to_id.items()};ir_goals=[list(inverse[v]) for v in ids]
 metadata={'case_id':row['case_id'],'initial_targets_differ':sum(x!=y for x,y in zip(cpp_goals,ir_goals)),'num_agents':row['num_agents'],'cpp_cost':json.loads(cpp.stdout)['cost'],'cpp_goals':cpp_goals,'ir_goals':ir_goals}
 (folder/'assignment_comparison.json').write_text(json.dumps(metadata,indent=2));results=[]
 for variant,goals in [('cpp_initial_fixed',cpp_goals),('ir_initial_fixed',ir_goals)]:
  if variant not in a.control_variants:continue
  assert len(goals)==len(data['agents']) and len({tuple(g) for g in goals})==len(goals)
  restricted=json.loads(json.dumps(data))
  for agent,goal in zip(restricted['agents'],goals):
   assert goal in agent['potentialGoals'];agent['potentialGoals']=[goal]
  target=folder/variant/'fixture';target.mkdir(parents=True,exist_ok=True)
  link=target/data['map']
  if not link.exists():link.symlink_to(map_path)
  newfixture=target/'case.yaml';newfixture.write_text(yaml.safe_dump(restricted,sort_keys=False))
  results.append(solve(row,variant,newfixture,cpu))
 return results
results=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
 for group in pool.map(work,enumerate(cases)):
  results.extend(group);print(f'{a.mode}: {len(results)} diagnostic records',flush=True)
summary={'mode':a.mode,'cases':len(cases),'runs':len(results),'validated_successes':dict(collections.Counter(r['variant'] for r in results if r['valid_solution_verified'])),'results':results}
(root/(a.variant_prefix+a.mode+'_summary.json')).write_text(json.dumps(summary,indent=2));print({k:v for k,v in summary.items() if k!='results'},flush=True)
