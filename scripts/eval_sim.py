"""Offline load, constrained motion, free fall/contact smoke tests and preview."""
import sys
sys.stdout.reconfigure(encoding='utf8',errors='replace')
from pathlib import Path
import argparse,json,math
import numpy as np
import os
_DLL_DIRECTORY = os.add_dll_directory(str(Path(sys.prefix)/'Lib/site-packages/mujoco')) if os.name=='nt' else None
if os.name=='nt':
 import ctypes
 _MJ_DLL=ctypes.WinDLL(str(Path(sys.prefix)/'Lib/site-packages/mujoco/mujoco.dll'))
import mujoco
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs';OUT.mkdir(exist_ok=True)
p=argparse.ArgumentParser();p.add_argument('--view',action='store_true');args=p.parse_args()
m=mujoco.MjModel.from_xml_path(str(ROOT/'assets/mushibot.xml'));d=mujoco.MjData(m)
mujoco.mj_forward(m,d)
print('LOADED',mujoco.__version__,'nbody',m.nbody,'nq',m.nq,'nv',m.nv,'nu',m.nu,'neq',m.neq,'nmesh',m.nmesh,'mass',m.body_mass.sum())
if args.view:
 import mujoco.viewer
 with mujoco.viewer.launch_passive(m,d) as viewer:
  import time
  while viewer.is_running():mujoco.mj_step(m,d);viewer.sync();time.sleep(m.opt.timestep)
 sys.exit()
report={'mujoco_version':mujoco.__version__,'nbody':m.nbody,'nq':m.nq,'nv':m.nv,'nu':m.nu,'neq':m.neq,'nmesh':m.nmesh,'mass_kg':float(m.body_mass.sum())}
# Pin floating chassis only in the TEST (model remains free base).
m.eq_active0[:]=True
m.opt.gravity[:]=0
initial=d.qpos.copy();sitepairs=[(m.site(s+'_closure_lower').id,m.site(s+'_closure_rod').id) for s in ['L','R']]
maxerr=0.;traces=[];hip_history=[];knee_history=[];render_states=[];dense_traces=[];self_contacts=0;min_contact_distance=0.;initial_contacts=int(d.ncon)
for k in range(3000):
 d.qpos[:7]=initial[:7];d.qvel[:6]=0
 d.ctrl[:]=[.12*math.sin(k*.001*2*math.pi),1.0,.10*math.sin(k*.001*2*math.pi+.3),-.8]
 mujoco.mj_step(m,d)
 mujoco.mj_forward(m,d)
 dense_traces.append({'t':d.time,'qpos':d.qpos.tolist()})
 for con in d.contact:
  names=[m.geom(int(g)).name for g in con.geom]
  if any('_outer_shell_contact_' in n for n in names):self_contacts+=1;min_contact_distance=min(min_contact_distance,float(con.dist))
 assert np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel))
 maxerr=max(maxerr,max(float(np.linalg.norm(d.site_xpos[a]-d.site_xpos[b])) for a,b in sitepairs))
 hip_history.append([float(d.joint(s+'_hip').qpos[0]) for s in ['L','R']]);knee_history.append([float(d.joint(s+'_knee').qpos[0]) for s in ['L','R']])
 if k%100==0:render_states.append(d.qpos.copy())
 if k%100==0:traces.append({'t':d.time,'qpos':d.qpos.tolist(),'closure_m':max(float(np.linalg.norm(d.site_xpos[a]-d.site_xpos[b])) for a,b in sitepairs)})
report['pinned_motion']={'steps':3000,'duration_s':float(d.time),'max_closure_error_m':maxerr,'hip_angles_rad':[float(d.joint(s+'_hip').qpos[0]) for s in ['L','R']],'wheel_angles_rad':[float(d.joint(s+'_wheel_joint').qpos[0]) for s in ['L','R']],'warnings':[int(x.number) for x in d.warning]}
report['selective_self_contact']={'initial_contacts':initial_contacts,'contact_instances':self_contacts,'minimum_proxy_distance_m':min_contact_distance}
(OUT/'motion-trace-dense.json').write_text(json.dumps(dense_traces))
report['pinned_motion']['hip_peak_to_peak_rad']=np.ptp(hip_history,axis=0).tolist()
report['pinned_motion']['knee_peak_to_peak_rad']=np.ptp(knee_history,axis=0).tolist()
assert min(np.ptp(hip_history,axis=0))>.1 and min(np.ptp(knee_history,axis=0))>.05
assert all(x.number==0 for x in d.warning)
# Independent free-base gravity smoke test, no balancing claim.
mujoco.mj_resetData(m,d);m.opt.gravity[:]=[0,0,-9.81];maxcontact=0;freeerr=0
for k in range(2000):
 d.ctrl[:]=0;mujoco.mj_step(m,d);assert np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel));maxcontact=max(maxcontact,d.ncon);freeerr=max(freeerr,max(float(np.linalg.norm(d.site_xpos[a]-d.site_xpos[b])) for a,b in sitepairs))
report['free_base_drop']={'steps':2000,'duration_s':float(d.time),'max_contacts':maxcontact,'final_base_z_m':float(d.qpos[2]),'max_closure_error_m':freeerr,'warnings':[int(x.number) for x in d.warning]}
report['render']={}
try:
 from PIL import Image
 mujoco.mj_resetData(m,d);mujoco.mj_forward(m,d)
 cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.075];cam.distance=.30;cam.azimuth=135;cam.elevation=-20
 with mujoco.Renderer(m,height=960,width=1280) as r:
  r.update_scene(d,camera=cam);img=r.render();Image.fromarray(img).save(OUT/'preview.png')
  frames=[]
  for state in render_states:
   d.qpos[:]=state;mujoco.mj_forward(m,d);r.update_scene(d,camera=cam);frames.append(Image.fromarray(r.render()).resize((640,480)))
  frames[0].save(OUT/'motion-preview.gif',save_all=True,append_images=frames[1:],duration=100,loop=0)
 report['render']={'ok':True,'path':'outputs/preview.png','shape':list(img.shape)}
except Exception as e:report['render']={'ok':False,'error':repr(e)}
(OUT/'validation.json').write_text(json.dumps(report,indent=2),encoding='utf8');(OUT/'motion-trace.json').write_text(json.dumps(traces,indent=2),encoding='utf8')
print(json.dumps(report,indent=2))
assert abs(m.body_mass.sum()-.336)<1e-9
assert maxerr<.001 and freeerr<.002
assert maxcontact>0
assert all(x.number==0 for x in d.warning)
assert report['render']['ok'], report['render']
print('PASS offline geometry/load/motion/contact smoke tests (NOT balance validation)')
