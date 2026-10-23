"""Free-slope geometry in one fixed INIT-up map gauge."""
import numpy as np


def fit_plane(points, up):
    p=np.asarray(points,float);up=np.asarray(up,float)
    if len(p)<8:return None
    c=p.mean(0);d=p-c;v,V=np.linalg.eigh(d.T@d/len(p));n=V[:,0]
    if n@up<0:n=-n
    residual=float(np.percentile(abs(d@n),95))
    if (v[1]<.0025 or v[0]/max(v[1],1e-12)>.08 or residual>.05
            or n@up<np.cos(np.deg2rad(35))):return None
    return dict(center=c,normal=n,residual_p95=residual,eigenvalues=v,count=len(p))


def height_at(fit, reference, up):
    up=np.asarray(up,float);reference=np.asarray(reference,float)
    # The line through reference parallel to fixed up intersects the free plane.
    return float(reference@up-fit['normal']@(reference-fit['center'])/(fit['normal']@up))


def angle(a,b):
    a=np.asarray(a,float);b=np.asarray(b,float)
    return float(np.degrees(np.arctan2(np.linalg.norm(np.cross(a,b)),a@b)))
