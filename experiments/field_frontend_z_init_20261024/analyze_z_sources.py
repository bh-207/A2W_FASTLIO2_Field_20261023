#!/usr/bin/env python3
"""Fixed-direction stage bookkeeping. Increments are not independent errors."""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def load(path):return list(csv.DictReader(path.open()))
def xyz(r,p='t'):return np.array([float(r[p+a]) for a in 'xyz'])
def angle(a,b):return np.rad2deg(np.arccos(np.clip(np.sum(a*b,axis=-1)/np.linalg.norm(a,axis=-1)/np.linalg.norm(b,axis=-1),-1,1)))
def analyze(run,out,up=None):
 out.mkdir(exist_ok=False,parents=True);stages=load(run/'reconstruction/z_stages.csv');group={}
 for row in stages:group.setdefault(int(row['source_scan_id']),{})[row['stage']]=row
 groups=[x for x in group.values() if 'POST_LIDAR_UPDATE'in x];first=groups[0]['PRE_PROPAGATION'];g0=xyz(first,'g');u=-g0/np.linalg.norm(g0) if up is None else np.array(up)
 p0=xyz(first);items=[];start=int(groups[0]['POST_LIDAR_UPDATE']['source_header_stamp_ns'])
 for rows in groups:
  pre=rows['PRE_PROPAGATION'];prop=rows['POST_PROPAGATION'];post=rows['POST_LIDAR_UPDATE'];p=xyz(post);g=xyz(post,'g')
  items.append(dict(source_scan_id=int(post['source_scan_id']),time_s=(int(post['source_header_stamp_ns'])-start)*1e-9,native_z=p[2],fixed_up_height=float(p@u),prop_dz=float((xyz(prop)-xyz(pre))[2]),lidar_dz=float((p-xyz(prop))[2]),prop_dh=float((xyz(prop)-xyz(pre))@u),lidar_dh=float((p-xyz(prop))@u),projection_difference=float((p-p0)[2]-(p-p0)@u),g_change_deg=float(angle(g,g0)),ba_norm=float(np.linalg.norm(xyz(post,'ba'))),bg_norm=float(np.linalg.norm(xyz(post,'bg')))))
 with(out/'per_scan.csv').open('w')as f:w=csv.DictWriter(f,items[0]);w.writeheader();w.writerows(items)
 a={k:np.array([r[k]for r in items])for k in items[0]};summary={'up_fixed':u.tolist(),'up_source':'baseline first initialized state; same direction for every candidate','post_count':len(items),'raw_z_span_m':float(np.ptp(a['native_z'])),'fixed_up_height_span_m':float(np.ptp(a['fixed_up_height'])),'projection_difference_range_m':[float(a['projection_difference'].min()),float(a['projection_difference'].max())],'g_change_max_deg':float(a['g_change_deg'].max()),'g_change_end_deg':float(a['g_change_deg'][-1]),'ba_max_norm_m_s2':float(a['ba_norm'].max()),'stage_sum_dh_m':{k:float(np.sum(a[k]))for k in ['prop_dh','lidar_dh']},'stage_total_variation_dh_m':{k:float(np.sum(np.abs(a[k])))for k in ['prop_dh','lidar_dh']},'warning':'Propagation and lidar increments are coupled estimator bookkeeping; signed sums and correlation do not identify physical error. Ground truth unknown.'}
 (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
 fig,ax=plt.subplots(3,1,figsize=(12,10),sharex=True);ax[0].plot(a['time_s'],a['native_z'],label='native z');ax[0].plot(a['time_s'],a['fixed_up_height'],label='fixed initial-up height');ax[0].set_ylabel('m');ax[1].plot(a['time_s'],np.cumsum(a['prop_dh']),label='cumulative propagation');ax[1].plot(a['time_s'],np.cumsum(a['lidar_dh']),label='cumulative lidar update');ax[1].set_ylabel('signed increment m');ax[2].plot(a['time_s'],a['g_change_deg'],label='g direction vs initial (deg)');ax[2].plot(a['time_s'],a['ba_norm'],label='ba norm (m/s²)');ax[2].set_xlabel('source seconds after first POST')
 for x in ax:x.grid();x.legend()
 fig.tight_layout();fig.savefig(out/'z_stage_sources.png',dpi=150);plt.close(fig);return summary
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('out',type=Path);p.add_argument('--up',nargs=3,type=float);a=p.parse_args();print(json.dumps(analyze(a.run,a.out,a.up),indent=2))
