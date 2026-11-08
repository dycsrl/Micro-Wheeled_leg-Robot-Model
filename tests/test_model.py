"""离线模型、资源与自由基座契约检查。"""
import unittest
from pathlib import Path
import json, xml.etree.ElementTree as ET
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
class ModelTests(unittest.TestCase):
 def test_assets(self):
  tree=ET.parse(ROOT/'assets/mushibot.xml')
  refs=[x.attrib['file'] for x in tree.findall('./asset/mesh')]
  self.assertEqual(len(refs),252)
  self.assertTrue(all((ROOT/'assets/meshes'/x).is_file() for x in refs))
  self.assertEqual(len(list((ROOT/'assets/meshes').glob('*.obj'))),len(refs))
 def test_contract(self):
  m=mujoco.MjModel.from_xml_path(str(ROOT/'assets/mushibot.xml'))
  self.assertEqual((m.nbody,m.nq,m.nv,m.nu,m.neq),(11,15,14,4,2))
  self.assertAlmostEqual(float(m.body_mass.sum()),0.336,places=10)
  self.assertEqual(m.jnt_type[0],mujoco.mjtJoint.mjJNT_FREE)
 def test_manifest(self):
  x=json.loads((ROOT/'docs/cad-manifest.json').read_text())
  self.assertEqual(len(x['parts']),156)
  self.assertEqual(len({p['solid'] for p in x['parts']}),156)
  self.assertNotIn(126,{p['solid'] for p in x['parts']})
 def test_step_finite(self):
  m=mujoco.MjModel.from_xml_path(str(ROOT/'assets/mushibot.xml'));d=mujoco.MjData(m)
  for _ in range(100):mujoco.mj_step(m,d)
  self.assertTrue(np.isfinite(d.qpos).all())
  self.assertTrue(all(x.number==0 for x in d.warning))
if __name__=='__main__':unittest.main()
