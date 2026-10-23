#!/usr/bin/env python3
"""Same frozen identities/gauge; figures are display only, not new metrics."""
import os
os.environ['MPLCONFIGDIR']='/tmp/a2w_backend_summary_mpl'
from pathlib import Path
import json,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'geometry'))
from evaluate import ROOT,FROZEN,BUNDLE,poses,read_csv,transform

def main():
    manifest=json.loads((FROZEN/'selection_manifest.json').read_text());basis=np.array(manifest['basis'])
    sources=[('Raw',BUNDLE/'keyframes/index.csv'),('Old optimized',BUNDLE/'optimized_poses.csv'),
             ('Candidate (FAIL)',ROOT.parent/'frozen/bundles/guarded_v2_candidate_run1/optimized_poses.csv')]
    variants=[(name,poses(read_csv(path))) for name,path in sources]
    regions=[r for r in manifest['protocol']['regions'] if r['name'] in ['corridor_Y_-125','corridor_Y_-225']]
    fig,axes=plt.subplots(2,3,figsize=(16,7.5),sharex=True,sharey=True)
    for row,r in enumerate(regions):
        data=np.load(FROZEN/(r['name']+'.npz'));center=np.array(r['center'])
        for col,(name,pose) in enumerate(variants):
            ax=axes[row,col]
            for group,color in [('train','#2878ba'),('heldout','#29944a'),('revisit','#e87825')]:
                q=transform(data,group,pose)[data[group+'_floor']];q=q[::max(1,len(q)//4500)]
                local=(q-center)@basis;ax.scatter(local[:,1],local[:,2],s=.5,alpha=.5,c=color,label=group)
            ax.set(xlim=(-6,6),ylim=(-1.2,.4),title=name+' | '+r['name'],xlabel='Fixed horizontal axis 2 (m)',ylabel='Fixed INIT-up height (m)');ax.grid(alpha=.2)
    axes[0,0].legend(markerscale=5);fig.suptitle('Same original records, fixed gauge; free slopes retained. Vertical axis magnified.');fig.tight_layout();fig.savefig(ROOT/'main_floor_comparison.png',dpi=150);plt.close(fig)
    report=json.loads((ROOT/'final_bundle/report.json').read_text());fig,axes=plt.subplots(1,2,figsize=(13,4.8))
    groups=report['geometry'][1:5];x=np.arange(4);width=.25
    for k,(label,key,color) in enumerate([('Raw','raw','#2878ba'),('Old optimized','old_optimized','#e87825'),('Candidate','final_bundle','#29944a')]):
        ys=[100*g['comparisons']['final_bundle']['variants'][key]['floor']['absolute_return_height_difference_m']['median'] for g in groups]
        axes[0].bar(x+(k-1)*width,ys,width,label=label,color=color)
    axes[0].set(xticks=x,xticklabels=['Y -75','Y -125','Y -175','Y -225'],ylabel='Cross-pass height median (cm)',title='Main floor seams: relative geometric proxy');axes[0].legend()
    axes[1].bar(['Old median','Candidate median','Old P95','Candidate P95'],[17.417,5.583,41.107,55.752],color=['#e87825','#29944a','#e87825','#b92739'])
    axes[1].set(ylabel='Wall point-to-local-plane distance (cm)',title='Wall Y -125: same 131 queries; FAIL');axes[1].tick_params(axis='x',rotation=12)
    fig.tight_layout();fig.savefig(ROOT/'improvement_and_failure.png',dpi=150);plt.close(fig)
    selection=np.load(ROOT/'viewer_point_selection.npz');body=selection['body'];ids=selection['scan_id']
    worlds=[np.einsum('nij,nj->ni',p[ids,:3,:3],body)+p[ids,:3,3] for _,p in variants]
    limits=np.concatenate(worlds);lo=limits.min(0)-1;hi=limits.max(0)+1
    fig,axes=plt.subplots(2,3,figsize=(17,8))
    for col,((name,_),q) in enumerate(zip(variants,worlds)):
        for row,a in enumerate([1,0]):
            ax=axes[row,col];ax.scatter(q[:,a],q[:,2],s=.4,c=ids,cmap='viridis',vmin=0,vmax=827)
            ax.set(xlim=(lo[a],hi[a]),ylim=(lo[2],hi[2]),xlabel='Map '+('Y' if a==1 else 'X')+' (m)',ylabel='Native map Z (m)',title=name);ax.grid(alpha=.2)
    fig.suptitle('Same 10,000 original point identities; common axes per row; vertical axis magnified. Native Z is not drift truth.');fig.tight_layout();fig.savefig(ROOT/'whole_native_yz_xz.png',dpi=150);plt.close(fig)

if __name__=='__main__':main()
