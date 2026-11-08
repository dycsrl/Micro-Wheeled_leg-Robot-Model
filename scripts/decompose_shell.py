import os,sys
from pathlib import Path
import coacd,trimesh,numpy as np
r=Path(__file__).resolve().parents[1]
coacd.set_log_level('warn')
for i in [25,27]:
 mesh=trimesh.load(r/f'assets/meshes/part_{i:03}.obj',force='mesh',process=True)
 print('DECOMPOSE',i,len(mesh.vertices),mesh.is_watertight,flush=True)
 pieces=coacd.run_coacd(coacd.Mesh(np.asarray(mesh.vertices)*1000,np.asarray(mesh.faces)),threshold=.03,max_convex_hull=48,preprocess_mode='auto',resolution=1500,mcts_nodes=15,mcts_iterations=100,mcts_max_depth=3,seed=0)
 for k,(v,f) in enumerate(pieces):trimesh.Trimesh(np.asarray(v)*.001,f,process=False).export(r/f'assets/meshes/contact_{i}_{k}.obj')
 print('HULLS',i,len(pieces),flush=True)
