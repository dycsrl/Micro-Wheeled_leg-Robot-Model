"""Deterministic floor regression using compiled visual vertices (metres)."""
import os,sys,json,math,argparse
from pathlib import Path
import numpy as np
if os.name=='nt':
 _dll=os.add_dll_directory(str(Path(sys.prefix)/'Lib/site-packages/mujoco'))
 import ctypes
 _lib=ctypes.WinDLL(str(Path(sys.prefix)/'Lib/site-packages/mujoco/mujoco.dll'))
import mujoco
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs';OUT.mkdir(exist_ok=True)
p=argparse.ArgumentParser();p.add_argument('--label',default='after');p.add_argument('--render',action='store_true');p.add_argument('--model');a=p.parse_args()
m=mujoco.MjModel.from_xml_path(str(Path(a.model) if a.model else ROOT/'assets/mushibot.xml'));d=mujoco.MjData(m)
floor=m.geom('floor').id
meshes=[]
for g in range(m.ngeom):
 if m.geom(g).name.startswith('visual_'):
  mid=m.geom_dataid[g];start=m.mesh_vertadr[mid];n=m.mesh_vertnum[mid]
  meshes.append((g,m.mesh_vert[start:start+n].astype(float).copy()))
def measure():
 mujoco.mj_forward(m,d)
 origin=d.geom_xpos[floor];normal=d.geom_xmat[floor].reshape(3,3)[:,2]
 vals=[]
 for g,v in meshes:
  rot=d.geom_xmat[g].reshape(3,3)
  z=v@(normal@rot)+(d.geom_xpos[g]-origin)@normal
  vals.append((float(z.min()),m.geom(g).name,m.body(int(m.geom_bodyid[g])).name))
 vals.sort();cs=[{'dist_m':float(c.dist),'geoms':[m.geom(int(g)).name for g in c.geom]} for c in d.contact if floor in c.geom]
 return vals,cs
report={'mujoco':mujoco.__version__,'visual_mesh_count':len(meshes),'mass_kg':float(m.body_mass.sum()),'floor_z_m':float(m.geom_pos[floor,2]),'sampling_s':.001,'scenarios':{}}
frames={}
for mode in ['pinned_animation','free_drop']:
 mujoco.mj_resetData(m,d);initial=d.qpos.copy();m.opt.gravity[:]=[0,0,0 if mode=='pinned_animation' else -9.81]
 worst=(1.,None,None);contact_min=None;max_contacts=0;bybody={};states=[];records=[];worstq=None;contactpair=None
 for k in range(5001 if mode=='free_drop' else 3001):
  if k%1==0:
   vals,contacts=measure();v,name,body=vals[0];max_contacts=max(max_contacts,len(contacts))
   for z,gn,bn in vals:bybody[bn]=min(bybody.get(bn,1.),z)
   if v<worst[0]:worst=(v,float(d.time),name);worstq=d.qpos.copy()
   for c in contacts:
    if contact_min is None or c['dist_m']<contact_min:contact_min=c['dist_m'];contactpair=c
   records.append({'t':float(d.time),'min_visual_height_m':v,'geom':name,'floor_contacts':len(contacts)})
   if k%100==0:states.append(d.qpos.copy())
  if mode=='pinned_animation':
   d.qpos[:7]=initial[:7];d.qvel[:6]=0;d.ctrl[:]=[.12*math.sin(k*.001*2*math.pi),1.,.10*math.sin(k*.001*2*math.pi+.3),-.8]
  else:d.ctrl[:]=0
  mujoco.mj_step(m,d)
  assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
 report['scenarios'][mode]={'min_visual_height_m':worst[0],'worst_time_s':worst[1],'worst_visual_geom':worst[2],'max_floor_contacts':max_contacts,'min_floor_contact_dist_m':contact_min,'worst_contact':contactpair,'per_body_min_visual_height_m':bybody,'warnings':[int(x.number) for x in d.warning],'samples':records,'worst_qpos':worstq.tolist()}
 frames[mode]=(states,worstq)
if a.render:
 from PIL import Image,ImageDraw
 cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.06];cam.distance=.32;cam.azimuth=130;cam.elevation=-18
 with mujoco.Renderer(m,height=720,width=960) as r:
  for mode,(states,worstq) in frames.items():
   images=[]
   for q in states:
    d.qpos[:]=q;mujoco.mj_forward(m,d);r.update_scene(d,camera=cam);im=Image.fromarray(r.render()).resize((640,480));images.append(im)
   images[0].save(OUT/f'ground-{a.label}-{mode}.gif',save_all=True,append_images=images[1:],duration=100,loop=0)
   d.qpos[:]=worstq;mujoco.mj_forward(m,d);r.update_scene(d,camera=cam);Image.fromarray(r.render()).save(OUT/f'ground-{a.label}-{mode}-worst.png')
path=OUT/f'ground-{a.label}.json';path.write_text(json.dumps(report,indent=2))
for mode,r in report['scenarios'].items():print(mode,json.dumps({k:v for k,v in r.items() if k not in ['samples','worst_qpos']}))
print('REPORT',path)
assert all(r['min_visual_height_m']>=-.001 for r in report['scenarios'].values()), 'Visual geometry penetrated floor by more than 1mm'
assert all(not any(r['warnings']) for r in report['scenarios'].values())
assert report['visual_mesh_count']==156
assert abs(report['mass_kg']-.336)<1e-9
assert report['scenarios']['free_drop']['max_floor_contacts']>0
assert report['scenarios']['free_drop']['min_floor_contact_dist_m']>=-.001
print('PASS visual floor clearance and finite-state regression')
