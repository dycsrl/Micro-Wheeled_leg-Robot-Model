import sys,os,json,math
from pathlib import Path
import numpy as np
import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
ROOT=Path(__file__).resolve().parents[1]
import argparse,hashlib
parser=argparse.ArgumentParser(description='对原始 STEP 做抽样实体求交；不是全行程证明')
parser.add_argument('--step',required=True,type=Path)
STEP=parser.parse_args().step.resolve()
assert STEP.is_file(), 'STEP 文件不存在'
OUT=ROOT/'outputs';OUT.mkdir(exist_ok=True)
if os.name=='nt':
 _DLL=os.add_dll_directory(str(Path(sys.prefix)/'Lib/site-packages/mujoco'))
 import ctypes
 _MJ=ctypes.WinDLL(str(Path(sys.prefix)/'Lib/site-packages/mujoco/mujoco.dll'))
import mujoco
sol=cq.importers.importStep(str(STEP)).solids().vals()
manifest=json.loads((ROOT/'docs/cad-manifest.json').read_text()); parts={p['solid']:p['body'] for p in manifest['parts']}
assert hashlib.sha256(STEP.read_bytes()).hexdigest()==manifest['source_sha256'], 'STEP 与建模来源不一致'
m=mujoco.MjModel.from_xml_path(str(ROOT/'assets/mushibot.xml'));d=mujoco.MjData(m)
R=np.array([[-1,0,0],[0,0,1],[0,1,0]],float);origin=np.array([0,32,15.5])
anchors={'chassis':np.zeros(3)}
for side,sign in [('L',1),('R',-1)]:
 for g,k in [('upper','A'),('lower','B'),('rod','C'),('wheel','W')]:anchors[side+'_'+g]=np.array(manifest['anchors_robot_m'][k])*[1,sign,1]
def transformed(i):
 group=parts[i];bid=m.body(group).id;rot=d.xmat[bid].reshape(3,3);pos=d.xpos[bid]-d.xpos[m.body('chassis').id]
 Q=R.T@rot@R;t=R.T@(-rot@origin-1000*rot@anchors[group]+1000*pos+origin)
 T=np.eye(4);T[:3,:3]=Q;T[:3,3]=t
 from OCP.gp import gp_Trsf
 from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
 tr=gp_Trsf();tr.SetValues(*T[:3,:].ravel().tolist())
 return cq.Shape.cast(BRepBuilderAPI_Transform(sol[i].wrapped,tr,True).Shape())
rows=[]
traces=json.loads((OUT/'motion-trace.json').read_text())
states=[('q0',m.qpos0.copy())]+[(f'animation_step_{k*100}',np.array(t['qpos'])) for k,t in enumerate(traces)]
for label,q in states:
 d.qpos[:]=q;mujoco.mj_forward(m,d)
 for tire,others in [(149,([24,25,137,28,146] if label=='q0' or int(label.split('_')[-1])%100==0 else [25])),(148,([26,27,138,29,147] if label=='q0' or int(label.split('_')[-1])%100==0 else [27]))]:
  a=transformed(tire)
  for other in others:
   b=transformed(other);dist=a.distance(b);v=abs(a.intersect(b).Volume()) if dist<1e-5 else 0
   rows.append(dict(state=label,tire=tire,other=other,distance_mm=dist,intersection_mm3=v))
 print(label,[(x['tire'],x['other'],round(x['distance_mm'],4),round(x['intersection_mm3'],4)) for x in rows[-10:]],flush=True)
(OUT/'interference-probe.json').write_text(json.dumps(rows,indent=2))
axis_records={}
for i in [18,19,20,21,24,25,26,27,137,138,148,149]:
 axes=[]
 for f in sol[i].Faces():
  ad=BRepAdaptor_Surface(f.wrapped)
  if ad.GetType()==GeomAbs_Cylinder:
   c=ad.Cylinder();p=c.Axis().Location();v=c.Axis().Direction()
   if abs(v.Z())>.99:axes.append([round(c.Radius(),6),*[round(x,6) for x in (p.X(),p.Y(),p.Z())]])
 axis_records[i]=axes
(OUT/'cylinder-axes.json').write_text(json.dumps(axis_records,indent=2))

assert len(rows)==310
assert max(x['intersection_mm3'] for x in rows)<=0.1
print('PASS 31 states / 310 targeted BREP pairs; not exhaustive swept-volume validation')
