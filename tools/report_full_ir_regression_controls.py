#!/usr/bin/env python3
"""Validate and display lower-load diagnostics without replacing benchmark records."""
import os
for name in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS']:os.environ[name]='1'
os.sched_setaffinity(0,set(range(124)))
import concurrent.futures,csv,html,json,subprocess,sys
from pathlib import Path
from report_paper_2609_14208 import verify_archived_schedule
repo=Path(__file__).resolve().parents[1];base=repo/'build/results';fixed=base/'paper_2609_14208_cache_fixed';controls=base/'paper_2609_14208_regression_controls';out=base/'paper_2609_14208_cache_comparison';old=base/'paper_2609_14208_repository'
read=lambda p:json.loads(p.read_text())
rows=[read(p) for p in (controls/'artifacts').glob('*/*/ir/result.json')];manifest=read(fixed/'regression_control_manifest.json');assert len(rows)==52 and {r['case_id'] for r in rows}=={c['case_id'] for c in manifest['cases']}
assert not any('infrastructure_error' in r for r in rows)
prov=read(controls/'provenance.json');assert prov['jobs']==8 and prov['binaries_sha256']['ir']==read(fixed/'provenance.json')['binaries_sha256']['ir']
original={r['case_id']:r for r in csv.DictReader((old/'rows.csv').open()) if r['method']=='ir'};main={r['case_id']:r for r in csv.DictReader((fixed/'rows.csv').open())}
for r in rows:
 m=main[r['case_id']];assert original[r['case_id']]['valid_solution']=='1' and m['valid_solution']=='0'
 for key in ['fixture_sha256','matrix_sha256','binary_sha256','map_sha256']:assert r[key]==m[key]
 assert r['time_limit']==10 and str(r['seed'])==m['seed']
success=[r for r in rows if r['valid_solution']]
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:errors=[e for e in pool.map(verify_archived_schedule,[(controls,r,32+i%8) for i,r in enumerate(success)]) if e]
assert not errors,errors
summary={'complete':True,'scope':'52 regression cases only; diagnostics excluded from full benchmark','cases':52,'solved':len(success),'still_failed':len(rows)-len(success),'archive_validation_errors':errors,'solver_jobs':8,'time_limit_sec':10,'binary_sha256':prov['binaries_sha256']['ir'],'recovered_by_map':{name:sum(r['valid_solution'] for r in rows if r['map']==name) for name in sorted({r['map'] for r in rows})}}
(controls/'coverage_audit.json').write_text(json.dumps(summary,indent=2));(out/'regression_control_summary.json').write_text(json.dumps(summary,indent=2))
link=old/'full_ir_regression_controls'
if not link.is_symlink():link.symlink_to(controls.resolve(),target_is_directory=True)
items=[]
for r in sorted(rows,key=lambda r:r['case_id']):
 case=html.escape(r['case_id']);items.append(f'<tr><td>{case}</td><td>成功</td><td>失败</td><td>{"成功" if r["valid_solution"] else "失败"}</td><td><a href="../full_ir_cache_fixed_runs/artifacts/{case}/ir/result.json">112 路记录</a> · <a href="../full_ir_regression_controls/artifacts/{case}/ir/result.json">8 路记录</a></td></tr>')
notice='<section id="load-diagnostics"><h2>并行负载诊断：51 个退化案例恢复</h2><p>全量重跑中，有 52 个 Common 案例从原成功变为失败，均位于 112 路阶段。使用相同修复版二进制、输入和 10 秒预算，改为 8 路并行复验，51 个 orz900d 案例全部成功；1 个 maze 案例仍快速退出。51 条诊断成功轨迹已独立核查。</p><p>这组结果表明 orz900d 失败对并行负载敏感。每例搜索预算按墙钟时间计，增加并行度能提高总吞吐，但也可能使单例在预算内搜索得更少。下面主统计仍使用完整重跑的单次结果，没有合并这 51 个诊断成功。</p><p><a href="regression_controls.html">打开 52 个测例对照</a> · <a href="regression_control_summary.json">诊断核查汇总</a> · <a href="../full_ir_cache_fixed_runs/parallelism_segments.json">并行度分段与测例清单</a></p></section>'
page='<!doctype html><meta charset="utf-8"><title>IR 并行负载诊断</title><style>body{font:16px system-ui;max-width:1400px;margin:30px auto;padding:20px}td,th{padding:10px;text-align:left;border-bottom:1px solid #ddd}table{width:100%;border-collapse:collapse}p{line-height:1.8}</style><p><a href="comparison.html">返回全量修复版对比</a></p>'+notice+'<table><tr><th>测例</th><th>原版</th><th>修复版 112 路</th><th>修复版 8 路</th><th>真实记录</th></tr>'+''.join(items)+'</table>'
(out/'regression_controls.html').write_text(page)
p=out/'comparison.html';text=p.read_text()
if 'id="load-diagnostics"' not in text:text=text.replace('<table><tr><th>实验</th>',notice+'<table><tr><th>实验</th>',1)
p.write_text(text)
p=fixed/'status.json';status=read(p);status.update(state='complete',audit_complete=True,archived_solutions_checked=read(fixed/'coverage_audit.json')['archived_solutions_checked'],comparison_report=str(out/'comparison.html'));p.write_text(json.dumps(status,indent=2))
completion=read(out/'completion_audit.json');completion['lower_load_diagnostic']=summary;(out/'completion_audit.json').write_text(json.dumps(completion,indent=2))
subprocess.run([sys.executable,str(repo/'tools/build_paper_comparison_page.py'),'--run-dir',str(old)],check=True)
print(summary)
