#!/usr/bin/env python3
"""Extend the existing Group curves; corrected IR exists only for 250–500."""
import os,json
os.environ['OPENBLAS_NUM_THREADS']='1'
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
repo=Path(__file__).resolve().parents[1];base=repo/'build/results';out=base/'group_250_500_cache_comparison'
cols=['suite','method','map','num_agents','valid_solution']
old=pd.read_csv(base/'paper_2609_14208_repository/rows.csv',usecols=cols);new=pd.read_csv(base/'group_250_500_repository/rows.csv',usecols=cols);fixed=pd.read_csv(base/'group_250_500_cache_fixed/rows.csv',usecols=cols)
raw=pd.concat([old,new]);raw=raw[(raw.suite=='group')&raw.method.isin(['lacam_dfs','ir'])].copy();raw['version']=raw.method.map({'lacam_dfs':'ITA-LaCAM','ir':'IR-TAPF (original cache)'})
fixed=fixed.copy();fixed['version']='IR-TAPF (cache fixed; 250–500 only)'
allrows=pd.concat([raw,fixed]);m=allrows.groupby(['map','num_agents','version']).agg(completed=('valid_solution','size'),solved=('valid_solution','sum')).reset_index();m['success_rate']=100*m.solved/m.completed
assert all(m.completed==20)
m.to_csv(out/'group_10_500_success_rates.csv',index=False)
fig,axes=plt.subplots(2,4,figsize=(16,7),sharex=True,sharey=True)
versions=[('ITA-LaCAM','#d62728','-'),('IR-TAPF (original cache)','#2ca02c','--'),('IR-TAPF (cache fixed; 250–500 only)','#1f77b4','-')]
maps=list(dict.fromkeys(c['map'] for c in json.loads((repo/'data/group_250_500_repository/manifest.json').read_text())['cases']))
for ax,mapname in zip(axes.flat,maps):
 ax.axvspan(225,510,color='#edf3fa');ax.axvline(225,color='#aab',linestyle=':',linewidth=1)
 for version,color,style in versions:
  s=m[(m['map']==mapname)&(m.version==version)]
  ax.plot(s.num_agents,s.success_rate,style,marker='o',markersize=3,linewidth=1.5,label=version,color=color)
 ax.set(title=mapname,xlim=(0,510),ylim=(-3,103),xticks=[0,100,200,300,400,500],xlabel='Number of agents',ylabel='Success rate (%)');ax.grid(alpha=.25)
axes[0,0].legend(fontsize=7);fig.suptitle('Group 10–500: shaded region is the new extension; corrected IR measured only at 250–500');fig.tight_layout();fig.savefig(out/'group_10_500_success.png',dpi=180);fig.savefig(out/'group_10_500_success.pdf');plt.close(fig)
print('Wrote full-range Group curves with explicit original/fixed IR labels')
