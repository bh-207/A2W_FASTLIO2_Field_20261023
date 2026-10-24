#!/usr/bin/env python3
"""Same source scan/point identities for visualization only, no geometry fitting."""
import argparse,csv,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).parent/'ground_eval'))
from pcd_io import read_cloud
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import plotly.graph_objects as go
from plotly.subplots import make_subplots

def collect(run):
 ix=run/'reconstruction/post_index.csv';points=[]
 for r in csv.DictReader(ix.open()):
  if int(r['source_scan_id'])%60:continue
  a,ids=read_cloud(ix.parent/r['cloud'],ix.parent/r['point_ids']);a=a[ids%23==0];R=Rotation.from_quat([float(r[k])for k in ['qx','qy','qz','qw']]).as_matrix();t=np.array([float(r[k])for k in ['tx','ty','tz']]);points.append(a@R.T+t)
 return np.vstack(points)
def views(root,out):
 out.mkdir(exist_ok=False,parents=True);names=['B_009','C_001','C_002'];data=[collect(root/'runs'/n)for n in names];lim=np.array([[min(x[:,i].min()for x in data),max(x[:,i].max()for x in data)]for i in range(3)]);labels=['Original complete-input','Joint requested: FALLBACK #1','Joint requested: FALLBACK #2'];fig=make_subplots(rows=1,cols=3,specs=[[{'type':'scene'}]*3],subplot_titles=labels)
 for i,a in enumerate(data):fig.add_trace(go.Scatter3d(x=a[:,0],y=a[:,1],z=a[:,2],mode='markers',marker=dict(size=1,color=a[:,2],colorscale='Viridis',cmin=lim[2,0],cmax=lim[2,1]),showlegend=False),row=1,col=i+1)
 scene=dict(aspectmode='data',camera=dict(eye=dict(x=1.7,y=.8,z=.5)),xaxis=dict(range=lim[0].tolist(),title='native X m'),yaxis=dict(range=lim[1].tolist(),title='native Y m'),zaxis=dict(range=lim[2].tolist(),title='native Z m'))
 fig.update_layout(height=720,title='Same scan IDs (%60=0), same preprocess point IDs (%23=0). No alignment or flattening.',scene=scene,scene2=scene,scene3=scene)
 fig.write_html(out/'common_points_3d.html',include_plotlyjs=True,full_html=True)
 for axes,label in [((1,2),'YZ'),((0,2),'XZ')]:
  f,ax=plt.subplots(3,1,figsize=(14,8),sharex=True,sharey=True)
  for pane,a,title in zip(ax,data,labels):pane.scatter(a[:,axes[0]],a[:,axes[1]],s=.3,c=a[:,2],vmin=lim[2,0],vmax=lim[2,1],cmap='viridis');pane.set_title(title);pane.set_xlim(lim[axes[0]]);pane.set_ylim(lim[axes[1]]);pane.set_aspect('equal',adjustable='box');pane.grid();pane.set_ylabel('Z m')
  ax[-1].set_xlabel(label[0]+' m');f.tight_layout();f.savefig(out/(label+'_same_scale.png'),dpi=170);plt.close(f)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('out',type=Path);a=p.parse_args();views(a.root,a.out)
