#!/usr/bin/env python3
"""score_f3dof.py - flatness check of a LiDAR estimate: does the replayed LiDAR map stay level, and how does it
compare with the camera map and the wheels? (methods from the LiDAR drift study, a1_tilt / a6b_fit)

  usage: score_f3dof.py <run> <lidar.tum> <lidar.tum.meta.json> <out.json>
  reads the run's results folder (RESULTS_DIR/series2_static/<run>); writes one JSON of numbers.
*Plain terms: a LiDAR map that thinks the floor tilts is not a trustworthy yardstick; this measures the tilt.*
"""
import os
import numpy as np, json, sys
from scipy.spatial.transform import Rotation as R
B=os.path.join(os.environ.get("RESULTS_DIR", os.path.expanduser("~/vslam-sidewalk-robot/results")), "series2_static") + "/"
def fit(P,Q):
    mp,mq=P.mean(0),Q.mean(0); H=(P-mp).T@(Q-mq); U,S,Vt=np.linalg.svd(H); Rm=Vt.T@U.T
    if np.linalg.det(Rm)<0: Vt[1]*=-1; Rm=Vt.T@U.T
    return (Rm@(P-mp).T).T+mq
def score(run, tum, meta):
    L=np.loadtxt(tum,comments='#'); W=np.loadtxt(B+run+"/wheel.tum",comments='#'); C=np.loadtxt(B+run+"/camera_corrected.tum",comments='#')
    m=json.load(open(meta))
    tilt=np.degrees(np.arccos(np.clip(1-2*(L[:,4]**2+L[:,5]**2),-1,1)))   # angle of body z from vertical
    gap_floor=float(np.linalg.norm(L[-1,1:3]-L[0,1:3])); gap3=float(np.linalg.norm(L[-1,1:4]-L[0,1:4]))
    k=(C[:,0]>=L[0,0])&(C[:,0]<=L[-1,0]); C=C[k]
    Pl=np.c_[np.interp(C[:,0],L[:,0],L[:,1]),np.interp(C[:,0],L[:,0],L[:,2])]
    el=np.linalg.norm(fit(Pl,C[:,1:3])-C[:,1:3],axis=1)
    k2=(W[:,0]>=L[0,0])&(W[:,0]<=L[-1,0]); W=W[k2][::10]
    Plw=np.c_[np.interp(W[:,0],L[:,0],L[:,1]),np.interp(W[:,0],L[:,0],L[:,2])]
    ew=np.linalg.norm(fit(Plw,W[:,1:3])-W[:,1:3],axis=1)
    hr=float(L[:,3].max()-L[:,3].min())
    return dict(nodes=len(L),closures=m.get("loop_closures"),gap_floor_m=round(gap_floor,2),gap_3d_m=round(gap3,2),
        tilt_max_deg=round(float(tilt.max()),1),z_min=round(float(L[:,3].min()),2),z_max=round(float(L[:,3].max()),2),height_range_m=round(hr,2),
        vs_camera_SE2_med_p95=[round(float(np.median(el)),2),round(float(np.percentile(el,95)),2)],camera_pairs=int(len(C)),
        vs_wheels_SE2_med_p95=[round(float(np.median(ew)),2),round(float(np.percentile(ew,95)),2)],
        flatness_pass=bool(tilt.max()<3 and hr<0.3))
if __name__=="__main__":
    run,tum,meta=sys.argv[1:4]; r=score(run,tum,meta); print(json.dumps(r))
    if len(sys.argv)>4: json.dump(r,open(sys.argv[4],"w"),indent=1)
