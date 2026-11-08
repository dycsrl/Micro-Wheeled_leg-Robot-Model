"""位置舵机离线承重回归；导轨锁横向与姿态，仅放开竖向，不是平衡测试。"""
import argparse
import hashlib
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
if os.name == 'nt':
    _dll = os.add_dll_directory(str(Path(sys.prefix)/'Lib/site-packages/mujoco'))
    import ctypes
    _lib = ctypes.WinDLL(str(Path(sys.prefix)/'Lib/site-packages/mujoco/mujoco.dll'))
import mujoco
ROOT = Path(__file__).resolve().parents[1]
SIDES = ('L', 'R')
LIMITS = dict(tail_error_rad=math.radians(2), closure_m=1e-4,
              torque_nm=.15, timestep_tail_difference_rad=.002,
              support_weight_relative_error=.03)


def model(guide=False, dt=.001, gains=None):
    root = ET.parse(ROOT/'assets/mushibot.xml').getroot()
    root.find('compiler').set('meshdir', (ROOT/'assets/meshes').as_posix())
    root.find('option').set('timestep', str(dt))
    if guide:
        body = root.find(".//body[@name='chassis']")
        body.remove(body.find('freejoint'))
        ET.SubElement(body, 'joint', name='bench_z', type='slide', axis='0 0 1', limited='false', damping='0')
    if gains:
        for side in SIDES:
            a = root.find(f"./actuator/position[@name='{side}_servo']")
            a.set('kp', str(gains[0])); a.set('kv', str(gains[1]))
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


def closure(m, d, side):
    return d.site(side+'_closure_lower').xpos - d.site(side+'_closure_rod').xpos


def pose(m, theta):
    """从CAD零位沿同一四杆分支求可行姿态，不借用训练工程的映射或物理参数。"""
    d = mujoco.MjData(m)
    for q in np.linspace(0, theta, 21):
        for side in SIDES:
            d.joint(side+'_hip').qpos[0] = q
            for _ in range(15):
                mujoco.mj_forward(m, d)
                r = closure(m, d, side).copy()
                if np.linalg.norm(r) < 1e-12:
                    break
                jac = []
                for name in ('knee', 'rod_pivot'):
                    j = d.joint(side+'_'+name)
                    old = float(j.qpos[0]); j.qpos[0] = old+1e-6
                    mujoco.mj_forward(m, d)
                    jac.append((closure(m, d, side)-r)/1e-6)
                    j.qpos[0] = old
                delta = np.linalg.lstsq(np.array(jac).T, -r, rcond=None)[0]
                for name, dq in zip(('knee', 'rod_pivot'), delta):
                    d.joint(side+'_'+name).qpos[0] += dq
    mujoco.mj_forward(m, d)
    dz = .0185-min(d.body(s+'_wheel').xpos[2] for s in SIDES)
    if m.joint(0).type[0] == mujoco.mjtJoint.mjJNT_FREE:
        d.qpos[2] += dz
    else:
        d.joint('bench_z').qpos[0] += dz
    for side in SIDES:
        d.ctrl[m.actuator(side+'_servo').id] = theta
    mujoco.mj_forward(m, d)
    assert max(np.linalg.norm(closure(m, d, s)) for s in SIDES) < 1e-9
    return d


def statics(theta):
    """联立导轨、闭环、双轮竖向支撑与锁轮静力方程，独立求左右扭矩。"""
    m = model(); d = pose(m, theta)
    rows = list(np.eye(m.nv)[[0, 1, 3, 4, 5]])
    supports = []
    for side in SIDES:
        a = np.zeros((3, m.nv)); b = a.copy()
        mujoco.mj_jacSite(m, d, a, None, m.site(side+'_closure_lower').id)
        mujoco.mj_jacSite(m, d, b, None, m.site(side+'_closure_rod').id)
        rows.extend(a-b)
        w = np.zeros((3, m.nv))
        mujoco.mj_jacBody(m, d, w, None, m.body(side+'_wheel').id)
        supports.append(len(rows)); rows.append(w[2])
        lock = np.zeros(m.nv); lock[m.joint(side+'_wheel_joint').dofadr[0]] = 1
        rows.append(lock)
    B = np.zeros((m.nv, 2))
    for i, side in enumerate(SIDES):
        B[m.joint(side+'_hip').dofadr[0], i] = 1
    A = np.column_stack((B, np.array(rows).T))
    sol = np.linalg.lstsq(A, d.qfrc_bias, rcond=1e-11)[0]
    derivatives = []
    potential = lambda x: float(np.sum(m.body_mass*x.xipos[:, 2])*9.81)
    for h in (1e-4, 1e-5, 1e-6):
        dp, dm = pose(m, theta+h), pose(m, theta-h)
        derivatives.append((potential(dp)-potential(dm))/(2*h))
    tangent = np.zeros(m.nv)
    mujoco.mj_differentiatePos(m, tangent, 2e-6, dm.qpos, dp.qpos)
    M = np.zeros((m.nv, m.nv)); mujoco.mj_fullM(m, M, d.qM)
    result = dict(theta_rad=theta, hip_height_m=float(d.body('chassis').xpos[2]),
                  torque_nm=sol[:2].tolist(), support_n=[float(sol[2+i]) for i in supports],
                  balance_residual=float(np.linalg.norm(A@sol-d.qfrc_bias)),
                  potential_derivatives_nm=derivatives,
                  virtual_work_error_nm=max(abs(sum(sol[:2])-v) for v in derivatives),
                  inertia_per_servo_kgm2=float(tangent@M@tangent/2))
    assert result['balance_residual'] < 1e-9
    assert result['virtual_work_error_nm'] < 1e-7
    assert max(abs(v) for v in sol[:2]) < .14709975
    return result


def bench(theta=-.2, delta=(0., 0.), dt=.001, gains=None, unloaded=False):
    m = model(True, dt, gains); d = pose(m, theta)
    if unloaded:
        # 独立/差动映射另用零重力悬轮测试；绝不把它叫作承重通过。
        m.opt.gravity[:] = 0
        d.joint('bench_z').qpos[0] += .1
        mujoco.mj_forward(m, d)
    assert float(m.opt.timestep) == dt
    aids = [m.actuator(s+'_servo').id for s in SIDES]
    target = np.full(2, theta); desired = target+delta
    maxclosure = 0.; maxforce = 0.; warnings = []; tail = []; trace = []
    for k in range(round(2/dt)):
        # 固定物理时刻启动，0.4rad/s目标斜坡为工程测试输入，不是舵机速度模型。
        if k*dt >= .5:
            target += np.clip(desired-target, -.4*dt, .4*dt)
        d.ctrl[aids] = target
        mujoco.mj_step(m, d); mujoco.mj_forward(m, d)
        assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
        maxclosure = max(maxclosure, *(float(np.linalg.norm(closure(m, d, s))) for s in SIDES))
        maxforce = max(maxforce, float(np.max(np.abs(d.actuator_force[aids]))))
        support = 0.; contacts = [0, 0]
        for conid, con in enumerate(d.contact):
            names = [m.geom(int(g)).name for g in con.geom]
            if 'floor' not in names:
                continue
            for i, s in enumerate(SIDES):
                if s+'_tire_contact' in names:
                    force = np.zeros(6); mujoco.mj_contactForce(m, d, conid, force)
                    support += float((con.frame.reshape(3, 3).T@force[:3])[2])
                    contacts[i] += 1
        q = np.array([float(d.joint(s+'_hip').qpos[0]) for s in SIDES])
        row = dict(t_s=float(d.time), target_rad=target.tolist(), hip_rad=q.tolist(),
                   support_n=support, wheel_contacts=contacts,
                   z_m=float(d.body('chassis').xpos[2]))
        if d.time > 1.7:
            tail.append(row)
        if (k+1) % round(.01/dt) == 0:
            trace.append(row)
    warnings = [int(w.number) for w in d.warning]
    result = dict(dt_s=float(m.opt.timestep), theta_rad=theta, delta_rad=list(delta),
                  tail_error_rad=max(float(np.max(np.abs(np.array(r['hip_rad'])-r['target_rad']))) for r in tail),
                  final_hip_rad=tail[-1]['hip_rad'], max_closure_m=maxclosure,
                  max_torque_nm=maxforce, warnings=warnings,
                  tail_support_n=float(np.mean([r['support_n'] for r in tail])),
                  min_tail_wheel_contacts=min(sum(r['wheel_contacts']) for r in tail),
                  compiled_gains=[[float(m.actuator_gainprm[i, 0]), float(-m.actuator_biasprm[i, 2])] for i in aids],
                  ctrlrange=m.actuator_ctrlrange[aids].tolist(), forcerange=m.actuator_forcerange[aids].tolist())
    result['unloaded'] = unloaded
    result['pass'] = bool(result['tail_error_rad'] < LIMITS['tail_error_rad'] and maxclosure < LIMITS['closure_m']
                          and maxforce <= LIMITS['torque_nm']+1e-9 and not any(warnings)
                          and (unloaded or (result['min_tail_wheel_contacts'] > 0
                          and abs(result['tail_support_n']/.336/9.81-1) < LIMITS['support_weight_relative_error'])))
    return result, trace


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--label', default='servo-hold')
    parser.add_argument('--candidates', action='store_true'); args = parser.parse_args()
    out = ROOT/'outputs'/args.label; out.mkdir(parents=True, exist_ok=True)
    states = [statics(t) for t in (-.25, -.2, -.1)]
    report = dict(model_sha256=hashlib.sha256((ROOT/'assets/mushibot.xml').read_bytes()).hexdigest(),
                  mujoco=mujoco.__version__, thresholds=LIMITS, statics=states, cases=[])
    if args.candidates:
        peak = max(abs(v) for s in states for v in s['torque_nm'])
        inertia = states[1]['inertia_per_servo_kgm2']
        report['candidates'] = []
        for deg in (2, 1):
            kp = 1.1*peak/math.radians(deg); kv = 2*math.sqrt(kp*inertia)
            r, _ = bench(gains=(kp, kv)); r.update(kp=kp, kv=kv, assumed_error_deg=deg)
            report['candidates'].append(r)
    cases = [(-.25, (0, 0)), (-.2, (0, 0)), (-.1, (0, 0)),
             (-.2, (-.04, 0)), (-.2, (0, -.04)), (-.2, (-.04, -.04)), (-.2, (-.03, .03))]
    for i, (theta, delta) in enumerate(cases):
        r, trace = bench(theta, delta)
        report['cases'].append(r)
        (out/f'trace-{i}.json').write_text(json.dumps(trace, indent=2), encoding='utf-8')
    fine, _ = bench(delta=(-.04, -.04), dt=.0005)
    difference = max(abs(a-b) for a, b in zip(fine['final_hip_rad'], report['cases'][5]['final_hip_rad']))
    report['half_step'] = fine; report['half_step_tail_difference_rad'] = difference
    # 导轨锁横滚且双轮等高接地，左右腿长不能独立变化：非同向目标过约束。
    # 保留其真实失败，另做悬轮映射检查；不扩大力矩、不放松承重静差门槛。
    report['unloaded_mapping'] = [bench(delta=delta, unloaded=True)[0]
                                  for delta in ((-.04, 0), (0, -.04), (-.03, .03))]
    report['loaded_asymmetric_tracking_pass'] = all(report['cases'][i]['pass'] for i in (3, 4, 6))
    report['pass'] = (all(report['cases'][i]['pass'] for i in (0, 1, 2, 5))
                      and all(r['pass'] for r in report['unloaded_mapping'])
                      and fine['pass'] and difference < LIMITS['timestep_tail_difference_rad'])
    (out/'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2), flush=True)
    assert report['pass'], '位置舵机承重/指令/步长回归未通过；详见summary.json'

if __name__ == '__main__':
    main()
