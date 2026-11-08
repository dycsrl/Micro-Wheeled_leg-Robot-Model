"""Rebuild real STEP tessellation + planar four-bar MJCF. Offline only."""
import sys
sys.stdout.reconfigure(encoding='utf8',errors='replace')
from pathlib import Path
import json,hashlib,math,xml.etree.ElementTree as E
import numpy as np
import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
ROOT=Path(__file__).resolve().parents[1]
import argparse
parser=argparse.ArgumentParser(description='由上游 STEP 重建模型；会更新 assets 与 CAD 清单')
parser.add_argument('--step',required=True,type=Path)
SOURCE=parser.parse_args().step.resolve()
assert SOURCE.is_file(), 'STEP 文件不存在'
(ROOT/'docs').mkdir(exist_ok=True)
CONFIG=json.loads((ROOT/'assets/model-parameters.json').read_text(encoding='utf8'))
R=np.array([[-1,0,0],[0,0,1],[0,1,0]],float); origin=np.array([0,32,15.5]) # transformed CAD reference
# CAD x,y,z -> robot -x,z-32,y-15.5 (mm -> m)
def xyz(p):return (R@np.array(p)-origin)*.001
def s(v):return ' '.join(f'{x:.10g}' for x in np.atleast_1d(v))
def sub(p,t,**kw):return E.SubElement(p,t,{k:str(v) for k,v in kw.items()})
solids=cq.importers.importStep(str(SOURCE)).solids().vals()
assert len(solids)==157
meshdir=ROOT/'assets/meshes';meshdir.mkdir(exist_ok=True,parents=True)
A=xyz([-4.793581489762,15.5,64]);B=xyz([43.843533312367,-3.139502775678,64]);C=xyz([12.901074593677,31.71416498278,64]);D=xyz([55.05965116221,1.126490705137,64]);W=xyz([-2.890291062,-20.914475612,75.65])
# Group membership follows STEP solid ordering, recorded in manifest.
groups={'chassis':list(range(18))+list(range(124,126))+list(range(127,131))+[136,137,138,139,140,143,144,145,150,151,156],
'L_upper':[22,24,25]+list(range(42,54))+list(range(66,78))+[78], 'R_upper':[23,26,27]+list(range(30,42))+list(range(54,66))+[79],
'L_lower':[18,142,146,132,153], 'R_lower':[20,141,147,134,155],
'L_rod':[28]+list(range(80,102))+[133,152], 'R_rod':[29]+list(range(102,124))+[131,135,154],
'L_wheel':[19,149], 'R_wheel':[21,148]}
# 126 is original battery, replaced by explicit user-sized, uncertain placement body.
flat=[i for g in groups.values() for i in g];assert len(set(flat))==len(flat);assert set(flat)==set(range(157))-{126}, sorted(set(range(157))-set(flat))
anchors={'chassis':np.zeros(3)}
for side in ['L','R']:
 sign=1 if side=='L' else -1
 def mir(p):q=p.copy();q[1]*=sign;return q
 anchors.update({side+'_upper':mir(A),side+'_lower':mir(B),side+'_rod':mir(C),side+'_wheel':mir(W)})
model=E.Element('mujoco',model='WL_PRO_STEP_fourbar')
sub(model,'compiler',angle='radian',meshdir='meshes',inertiafromgeom='false',autolimits='true')
sub(model,'option',timestep='.001',integrator='implicitfast',iterations='100',gravity='0 0 -9.81',solver='Newton',tolerance='1e-10')
vis=sub(model,'visual');sub(vis,'global',offwidth='1280',offheight='960');sub(vis,'headlight',ambient='.5 .5 .5',diffuse='.7 .7 .7')
default=sub(model,'default');sub(default,'joint',damping='.00005',armature='.0000001');sub(default,'geom',contype='0',conaffinity='0',density='0')
asset=sub(model,'asset');world=sub(model,'worldbody');sub(world,'light',pos='.2 -.2 .5',dir='-.2 .2 -.5');sub(world,'geom',name='floor',type='plane',size='1 1 .01',rgba='.16 .19 .23 1',contype='2',conaffinity='1',friction='.8 .01 .001')
base=sub(world,'body',name='chassis',pos='0 0 .09');sub(base,'freejoint',name='floating_base')
bodies={'chassis':base};eq=sub(model,'equality');acts=sub(model,'actuator')
for side in ['L','R']:
 sign=1 if side=='L' else -1
 a,b,c,d,w=[p*np.array([1,sign,1]) for p in [A,B,C,D,W]]
 up=sub(base,'body',name=side+'_upper',pos=s(a));sub(up,'joint',name=side+'_hip',axis='0 1 0',range='-.35 .35');bodies[side+'_upper']=up
 low=sub(up,'body',name=side+'_lower',pos=s(b-a));sub(low,'joint',name=side+'_knee',axis='0 1 0');bodies[side+'_lower']=low
 rod=sub(base,'body',name=side+'_rod',pos=s(c));sub(rod,'joint',name=side+'_rod_pivot',axis='0 1 0');bodies[side+'_rod']=rod
 sub(low,'site',name=side+'_closure_lower',pos=s(d-b),size='.001',rgba='1 .4 0 1');sub(rod,'site',name=side+'_closure_rod',pos=s(d-c),size='.001')
 sub(eq,'connect',name=side+'_fourbar',site1=side+'_closure_lower',site2=side+'_closure_rod',solref='.003 1',solimp='.99 .999 .0001')
 wheel=sub(low,'body',name=side+'_wheel',pos=s(w-b));sub(wheel,'joint',name=side+'_wheel_joint',axis='0 1 0');bodies[side+'_wheel']=wheel
 sub(wheel,'geom',name=side+'_tire_contact',type='cylinder',size='.0185 .0065',quat='.7071067812 .7071067812 0 0',rgba='0 0 0 0',contype='1',conaffinity='2',group='3',friction='.8 .01 .001')
 sub(acts,'position',name=side+'_servo',joint=side+'_hip',kp='6.0830245282658',kv='0.13525675068501658',ctrlrange='-.3 .3',forcerange='-.15 .15')
 # voltage input V -> torque estimated gain and back-emf damping. NOT calibrated.
 sub(acts,'general',name=side+'_wheel_voltage',joint=side+'_wheel_joint',gainprm='.01',biastype='affine',biasprm='0 0 -.0001',ctrlrange='-7.4 7.4',forcerange='-.08 .08')
rawmass={};records=[]
for group,ids in groups.items():
 rawmass[group]=0
 for i in ids:
  shape=solids[i];vol=shape.Volume()*1e-9
  density=1200 if i not in [18,19,20,21,148,149] and i>=18 else 3500
  if i in [148,149]:density=1100
  rawmass[group]+=vol*density
scale=.278/sum(rawmass.values())
colors={'chassis':'.30 .34 .39 1','upper':'.85 .68 .18 1','lower':'.28 .53 .75 1','rod':'.8 .8 .82 1','wheel':'.12 .13 .15 1'}
for group,ids in groups.items():
 anchor=anchors[group];body=bodies[group];total=0.;first=np.zeros(3);Iorigin=np.zeros((3,3))
 for i in ids:
  shape=solids[i];vol=shape.Volume()*1e-9;density=1200 if i not in [18,19,20,21,148,149] and i>=18 else 3500
  if i in [148,149]:density=1100
  mass=vol*density*scale;c=xyz(shape.Center().toTuple())-anchor
  J=R@np.array(cq.Shape.matrixOfInertia(shape))@R.T*1e-15*density*scale
  Iorigin+=J+mass*((c@c)*np.eye(3)-np.outer(c,c));total+=mass;first+=mass*c
  verts,tri=shape.tessellate(.15,.2);file=f'part_{i:03}.obj'
  with (meshdir/file).open('w') as f:
   for v in verts:f.write('v '+s(xyz(v.toTuple())-anchor)+'\n')
   for t in tri:f.write('f '+' '.join(str(j+1) for j in t)+'\n')
  sub(asset,'mesh',name=f'part_{i}',file=file)
  sub(body,'geom',name=f'visual_{i}',type='mesh',mesh=f'part_{i}',rgba=colors.get(group.split('_')[-1],colors['chassis']),group='1')
  records.append(dict(solid=i,body=group,file='meshes/'+file,volume_mm3=shape.Volume(),mass_kg_estimated=mass,vertices=len(verts),triangles=len(tri)))
 com=first/total;J=Iorigin-total*((com@com)*np.eye(3)-np.outer(com,com))
 sub(body,'inertial',pos=s(com),mass=f'{total:.12g}',fullinertia=s([J[0,0],J[1,1],J[2,2],J[0,1],J[0,2],J[1,2]]))
# Battery is the only synthetic box, explicit measured dimensions not substitute mechanism.
bat=sub(base,'body',name='battery_estimate',pos=s(CONFIG['battery_center_m']))
sub(bat,'geom',name='battery_measured_box',type='box',size='.01325 .0325 .0075',rgba='.25 .65 .3 .65',group='1')
m=.058;dims=np.array([.0265,.065,.015]);inertia=m/12*np.array([dims[1]**2+dims[2]**2,dims[0]**2+dims[2]**2,dims[0]**2+dims[1]**2]);sub(bat,'inertial',pos='0 0 0',mass='.058',diaginertia=s(inertia))
# Chassis floor contacts: real STEP mesh hulls, only static plates, no self-contact.
for i in [124,125,127,128,140]:sub(base,'geom',name=f'chassis_contact_{i}',type='mesh',mesh=f'part_{i}',rgba='0 0 0 0',contype='1',conaffinity='2',group='3')
# Every STEP visual needs floor coverage, including rods and lower legs when
# the unbalanced free base falls. A convex hull has the SAME support height
# against an infinite plane as the original vertices, even for concave shells.
# These copies only collide with floor (bit 2), never with robot bit 1.
for group,ids in groups.items():
 if group.endswith('_wheel'):continue  # existing tire cylinders cover wheels
 for i in ids:
  sub(bodies[group],'geom',name=f'floor_contact_{i}',type='mesh',mesh=f'part_{i}',rgba='0 0 0 0',contype='1',conaffinity='2',group='3',solref='.002 1',solimp='.99 .999 .0001',margin='.0005')
sub(bat,'geom',name='battery_floor_contact',type='box',size='.01325 .0325 .0075',rgba='0 0 0 0',contype='1',conaffinity='2',group='3',solref='.002 1',solimp='.99 .999 .0001',margin='.0005')
# Use the same finite, stiff floor-contact law on pre-existing proxies.
for geom in world.iter('geom'):
 if geom.get('name')=='floor' or geom.get('name','').endswith('_tire_contact') or geom.get('name','').startswith('chassis_contact_'):
  geom.set('solref','.002 1');geom.set('solimp','.99 .999 .0001');geom.set('margin','.0005')
# Selective non-joint tire / outer-thigh contact. Concave shell cannot be
# represented by its single convex hull (that would fill its large recess).
contact=sub(model,'contact')
for side,i in [('L',25),('R',27)]:
 files=sorted(meshdir.glob(f'contact_{i}_*.obj'))
 assert files, 'Run scripts/decompose_shell.py before rebuilding'
 for k,file in enumerate(files):
  name=f'{side}_outer_shell_contact_{k}'
  sub(asset,'mesh',name=name,file=file.name)
  sub(bodies[side+'_upper'],'geom',name=name,type='mesh',mesh=name,rgba='0 0 0 0',group='3')
  sub(contact,'pair',geom1=name,geom2=side+'_tire_contact',condim='3',friction='.6 .6 .001 .0001 .0001',solref='.002 1',solimp='.99 .999 .0001',margin='.0002')
E.indent(model);E.ElementTree(model).write(ROOT/'assets/mushibot.xml',encoding='utf-8',xml_declaration=True)
manifest={'source':SOURCE.name,'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),'cad_units':'mm','robot_frame':'X=-CAD_X; Y=CAD_Z-32mm; Z=CAD_Y-15.5mm','step_solids':len(solids),'visual_solids':len(records),'omitted_original_battery_solid':126,'anchors_robot_m':{k:v.tolist() for k,v in zip('ABCDW',[A,B,C,D,W])},'lengths_mm':{k:float(np.linalg.norm(v)*1000) for k,v in {'AB':B-A,'CD':D-C,'BD':D-B,'AC':C-A,'BW':W-B}.items()},'mass_kg':.336,'nonbattery_mass_method':'density-class volume weighting normalized to 0.278kg; unmeasured','parts':records}
(ROOT/'docs/cad-manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
print(json.dumps({k:v for k,v in manifest.items() if k!='parts'},indent=2));print('WROTE',ROOT/'assets/mushibot.xml')
