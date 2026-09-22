#!/usr/bin/env python3
"""Read-only dynamic HC force/torque comparison. Never imports ROS or commands hardware.

Outputs are offline candidates, not controller calibration. All source logs are immutable.
"""
import argparse
import csv
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.ndimage import uniform_filter1d
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'src/hc10dtp_bringup/scripts'))
from local_ik_solver import LocalIKSolver

JOINTS = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
# User supplied Axia -> flange = (0,-120,0) mm, ignoring spacer thickness.
# Interpret those axes as physical Axia measurement axes, with existing yaw correction.
FLANGE_TO_AXIA_SENSOR_M = np.array([0., .120, 0.])
TRIALS = [
    ('20260915_141843_943366', 'GT', 'Home X'),
    ('20260916_142750_583700_home_y', 'GT', 'Home Y'),
    ('20260917_093158_901998_home_z', 'GT', 'Home Z'),
    ('20260917_101035_730907_target1_x', 'GT', 'T1 X'),
    ('20260917_102540_949607_target1_y', 'GT', 'T1 Y'),
    ('20260917_104200_811194_target1_z', 'GT', 'T1 Z'),
    ('20260917_164311_423088_gru_home_to_target1', 'GRU', 'GRU T1'),
    ('20260917_165901_639040_gru_home_to_target2', 'GRU', 'GRU T2 before fault'),
    ('20260917_173750_951557_gru_mjm_home_to_target2', 'GRU+MJM', 'MJM T2 17:37'),
    ('20260917_174621_964444_gru_mjm_home_to_target2', 'GRU+MJM', 'MJM T2 17:46'),
]


def stats(x):
    x = np.asarray(x)
    x = x[np.isfinite(x)]
    return dict(n=len(x), median=float(np.median(x)), p95=float(np.percentile(x, 95)),
                maximum=float(np.max(x))) if len(x) else None


def metrics(pred, ref):
    if not len(ref):
        return {'n': 0}
    pn, rn = np.linalg.norm(pred, axis=1), np.linalg.norm(ref, axis=1)
    eligible = rn >= 4
    valid = eligible & (pn >= 4)
    dot = np.sum(pred * ref, axis=1)
    angles = np.degrees(np.arccos(np.clip(dot[valid] / (pn[valid]*rn[valid]), -1, 1)))
    return dict(n=len(ref), rmse_vector_n=float(np.sqrt(np.mean(np.sum((pred-ref)**2, axis=1)))),
                rmse_axes_n=np.sqrt(np.mean((pred-ref)**2, axis=0)).tolist(),
                reference_ge4=int(eligible.sum()), angle_n=int(valid.sum()),
                angle_coverage=float(valid.sum()/eligible.sum()) if eligible.any() else None,
                angle_deg=stats(angles), ref_norm_n=stats(rn),
                direction_failure_fraction=float(np.mean((pn[eligible] < 4) | (dot[eligible] <= 0)))
                if eligible.any() else None)


def interp(t, series, max_gap=.2):
    """Linear interpolation, explicitly reject extrapolation and missing intervals."""
    s = np.asarray(series)
    idx = np.searchsorted(s[:, 0], t, side='right')
    valid = (idx > 0) & (idx < len(s))
    idx = np.clip(idx, 1, len(s)-1)
    valid &= (s[idx, 0]-s[idx-1, 0]) <= max_gap
    values = np.column_stack([np.interp(t, s[:, 0], s[:, k]) for k in range(1, s.shape[1])])
    return values, valid


def transform_chain(edges, child, parent='base_link'):
    result = np.eye(4)
    seen = set()
    while child != parent:
        if child in seen or child not in edges:
            raise ValueError('Missing/cyclic TF chain: '+child)
        seen.add(child)
        child, mat = edges[child]
        result = mat @ result
    return result


def status_at(t, changes, default):
    idx = np.searchsorted([v[0] for v in changes], t, side='right')-1
    return np.array([changes[i][1] if i >= 0 else default for i in idx])


def parse_trial(path, mode, label):
    regs = {}; raw = []; joints = []; edges = {}; tf_checks = []
    states = []; run_flags = []; markers = []; complete = set(); latency = []
    failed = np.inf; counts = Counter(); t0 = None; args = None
    for line in (path/'events.jsonl').open():
        e = json.loads(line)
        t0 = e['monotonic_ns']/1e9 if t0 is None else t0
        t = e['monotonic_ns']/1e9-t0
        kind = e['kind']; counts[kind] += 1
        if kind == 'start': args = e.get('arguments', {})
        if 'timeout' in kind or kind == 'register_error': failed = min(failed, t)
        if kind == 'marker' and e.get('command', ''):
            markers.append((t, e['command']))
        if kind == 'scan_end': complete.add(e['scan_id'])
        if kind == 'register' and e.get('response', {}).get('success') and not e.get('late'):
            sent = e.get('sent_monotonic_ns', e['monotonic_ns'])/1e9-t0
            tm = (t+sent)/2
            regs.setdefault(e['address'], []).append((tm, (e['response']['value']-10000)*.1, e['scan_id']))
            latency.append(t-sent)
        if kind != 'topic': continue
        m = e['message']; topic = e['topic']
        if topic == '/cocarry/status': states.append((t, m['data']))
        if topic == '/run_status': run_flags.append((t, bool(m['data'])))
        if topic == '/joint_states' and all(n in m['name'] for n in JOINTS):
            joints.append([t, *[m['position'][m['name'].index(n)] for n in JOINTS]])
        if topic == '/axia/raw_wrench':
            raw.append([t, *[m['wrench'][part][a] for part in ['force', 'torque'] for a in 'xyz']])
        if topic in ['/tf', '/tf_static']:
            for tf in m['transforms']:
                tr = tf['transform']; mat = np.eye(4)
                mat[:3, :3] = Rotation.from_quat([tr['rotation'][a] for a in 'xyzw']).as_matrix()
                mat[:3, 3] = [tr['translation'][a] for a in 'xyz']
                edges[tf['child_frame_id']] = (tf['header']['frame_id'], mat)
            if not tf_checks or t-tf_checks[-1][0] >= .5:
                try: tf_checks.append((t, transform_chain(edges, 'tool0').copy()))
                except ValueError: pass
    raw = np.asarray(raw); joints = np.asarray(joints)
    baseline_idx = next(i for i, (_, ph) in enumerate(markers) if ph == 'baseline')
    lo = markers[baseline_idx][0]+1
    hi = markers[baseline_idx+1][0]-1
    if hi-lo < 2: raise ValueError('Initial no-contact baseline too short')
    base = (raw[:, 0]>=lo)&(raw[:, 0]<=hi)
    qraw, qvalid = interp(raw[:, 0], joints, .2)
    solver = LocalIKSolver()
    static = transform_chain(edges, 'axia_sensor_link', 'tool0')
    correction = Rotation.from_euler('z', -90, degrees=True).as_matrix().T
    # FK at every raw sample; avoids stale TF and interpolating rotations element-wise.
    poses = np.array([solver.forward_kinematics(q) for q in qraw])
    rots = poses[:, :3, :3] @ static[:3, :3] @ correction
    fg = np.einsum('nji,j->ni', rots, [0, 0, -9.80665*1.126])
    force_sensor = raw[:, 1:4]-fg
    force_sensor -= np.median(force_sensor[base & qvalid], axis=0)
    moment_sensor = raw[:, 4:7]-np.median(raw[base & qvalid, 4:7], axis=0)
    force = np.einsum('nij,nj->ni', rots, force_sensor)
    moment = np.einsum('nij,nj->ni', rots, moment_sensor)
    offset = np.einsum('nij,j->ni', rots, FLANGE_TO_AXIA_SENSOR_M)
    moment += np.cross(offset, force)
    wrench = np.column_stack((force, moment))
    # Same short symmetric smoothing for all Axia components; explicitly offline.
    window = max(1, int(round(.10/np.median(np.diff(raw[:, 0])))))
    smooth = uniform_filter1d(wrench, size=window, axis=0, mode='nearest')
    raw_series = np.column_stack((raw[:, 0], smooth))
    raw_series[~qvalid, 1:] = np.nan
    q0 = np.median(qraw[base & qvalid], axis=0)
    # Independent FK/TF consistency check at recorded timestamps (receive-time jitter remains).
    qt, valid_tf = interp(np.array([t for t, _ in tf_checks]), joints)
    fk_error = [np.linalg.norm(solver.fk_position(q)-tr[:3, 3])
                for q, (_, tr), valid in zip(qt, tf_checks, valid_tf) if valid]
    has_tau = all(a in regs for a in range(310, 316))
    needed = list(range(320, 323)) + (list(range(310, 316)) if has_tau else [])
    times = np.array([r[0] for r in regs[321] if r[2] in complete])
    data = {}; valid = times < failed
    for address in needed:
        s = np.asarray(regs[address]); s = s[np.array([int(i) in complete for i in s[:, 2]])]
        val, good = interp(times, s[:, :2], 1.5)
        data[address] = val[:, 0]; valid &= good
    w, good = interp(times, raw_series, .2); valid &= good & np.isfinite(w).all(axis=1)
    q, good = interp(times, joints, .2); valid &= good
    state = status_at(times, states, '')
    running = np.array([s.startswith('RUNNING') for s in state])
    running &= status_at(times, run_flags, True)
    phase = status_at(times, markers, 'setup')
    if label == 'T1 Z':
        valid &= ~np.isin(phase, ['zplus_3', 'release_zplus_3', 'baseline_end'])
    # End initial baseline before evaluating; preserves force transients, unlike static fit.
    valid &= times > hi
    goodrun = valid & running & ~np.isin(phase, ['baseline', 'baseline_end', 'stopped'])
    regbase = {}
    for address in needed:
        s = np.asarray(regs[address]); mask = (s[:, 0]>=lo)&(s[:, 0]<=hi)
        if mask.sum() < 3: raise ValueError('Missing baseline registers')
        regbase[address] = np.median(s[mask, 1])
    f = np.column_stack([data[a]-regbase[a] for a in range(320, 323)])
    tau = np.column_stack([data[a]-regbase[a] for a in range(310, 316)]) if has_tau else None
    jac = np.array([solver.compute_jacobian(v) for v in q])
    # Baseline torque changes with pose are NOT assumed to be already removed.
    target_tau = np.einsum('nji,nj->ni', jac, w)
    position = np.array([solver.fk_position(v) for v in q])
    speed = np.linalg.norm(np.gradient(position, times, axis=0), axis=1)
    scaled_j = jac.copy(); scaled_j[:, :3] /= .3
    condition = np.linalg.cond(scaled_j)
    angle = Rotation.from_matrix(rots[base & qvalid][0].T @ rots[qvalid]).magnitude()
    end = (status_at(raw[:, 0], markers, 'setup') == 'baseline_end') & qvalid
    end_summary = None
    if end.sum() > 100:
        end_summary = dict(axia_force_mean_n=np.mean(force[end], axis=0).tolist(),
                           axia_force_std_n=np.std(force[end], axis=0).tolist(),
                           joint_delta_rad=(np.median(qraw[end], axis=0)-q0).tolist())
    # CoG correction sensitivity; this is NOT a newly validated CoG.
    r_cog = np.array([-.00661, -.00098, .00824])
    gravity_m = np.cross(r_cog, fg)
    gravity_m -= np.median(gravity_m[base & qvalid], axis=0)
    cog_delta = np.einsum('nij,nj->ni', rots, gravity_m)
    # Translational inertia scale only; not used to silently relabel reference as pure contact force.
    position_raw = poses[:, :3, 3]
    from scipy.signal import savgol_filter
    dt = np.median(np.diff(raw[:, 0])); win = max(5, int(.5/dt)//2*2+1)
    inertial_scale = 1.126*np.linalg.norm(savgol_filter(position_raw, win, 3, deriv=2, delta=dt, axis=0), axis=1)
    meta = dict(trial=path.name, path=str(path.relative_to(ROOT)), mode=mode, label=label,
                declared_arguments=args, successful_register_counts={str(k):len(v) for k,v in regs.items()},
                has_tau=has_tau, counts=dict(counts), paired_running_samples=int(goodrun.sum()),
                moving_samples=int((goodrun & (speed>.005)).sum()),
                baseline_window_s=[lo,hi], baseline_joint_range_rad=np.ptp(qraw[base & qvalid], axis=0).tolist(),
                baseline_axia_force_std_n=np.std(force[base & qvalid], axis=0).tolist(),
                baseline_axia_vector_scatter_n=float(np.sqrt(np.mean(np.sum(force[base & qvalid]**2,axis=1)))),
                baseline_register={str(k):float(v) for k,v in regbase.items()},
                end_baseline=end_summary, sample_interval_s=stats(np.diff(times)), service_latency_s=stats(latency),
                fk_vs_recorded_tf_position_m=stats(fk_error), static_tool0_sensor_transform=static.tolist(),
                analysis_flange_to_axia_sensor_axes_m=FLANGE_TO_AXIA_SENSOR_M.tolist(),
                orientation_excursion_deg=stats(np.degrees(angle)),
                assumed_cog_moment_correction_nm=stats(np.linalg.norm(cog_delta, axis=1)),
                approximate_translational_inertial_force_n=stats(inertial_scale[qvalid]),
                jacobian_condition=stats(condition[goodrun]), markers=markers,
                source_sha256=hashlib.sha256((path/'events.jsonl').read_bytes()).hexdigest())
    scan_sets={}
    for address,series in regs.items():
        for tm,_,sid in series:
            if sid in complete:scan_sets.setdefault(sid,{})[address]=tm
    meta['sequential_register_span_s']={name:stats([max(s[a] for a in aa)-min(s[a] for a in aa)
        for s in scan_sets.values() if all(a in s for a in aa)])
        for name,aa in [('force',[320,321,322]),('torque',list(range(310,316))),('both',needed)]}
    print(label, 'samples',goodrun.sum(),'tau',has_tau,'FK',meta['fk_vs_recorded_tf_position_m'],flush=True)
    idx = goodrun
    return dict(meta=meta, t=times[idx], f=f[idx], tau=tau[idx] if has_tau else None,
                audit_samples=dict(t=times, q=q, tau=tau, w=w, valid=valid,
                                   running=running, phase=phase),
                q=q[idx], dq=q[idx]-q0, w=w[idx], target_tau=target_tau[idx], j=jac[idx],
                speed=speed[idx], phase=phase[idx], raw_series=raw_series, joints=joints,
                q0=q0, tool_rot=np.array([solver.forward_kinematics(v)[:3,:3] for v in q[idx]]),
                sensor_rot=np.array([solver.forward_kinematics(v)[:3,:3] @ static[:3,:3] @ correction for v in q[idx]]))


def fit_model(trials, branch, variant, lag=0.):
    xx=[]; yy=[]; weights=[]
    for d in trials:
        x = d['f'] if branch == 'force' else d['tau']
        y = d['w'][:, :3] if branch == 'force' else d['target_tau']
        if lag:
            key=('lag',lag)
            if key not in d:
                ww,good=interp(d['t']-lag,d['raw_series'])
                qq,qgood=interp(d['t']-lag,d['joints']);good &=qgood & np.isfinite(ww).all(axis=1)
                jj=np.array([LocalIKSolver().compute_jacobian(q) for q in qq])
                d[key]=(ww,good,jj)
            ww,good,jj=d[key]
            if branch == 'torque':
                y=np.einsum('nji,nj->ni',jj,ww)
            else: y=ww[:,:3]
        else: good=np.ones(len(x),dtype=bool)
        if variant == 'pose': x = np.column_stack((x, d['dq']))
        xx.append(x[good]); yy.append(y[good]); weights.extend([1/max(1,good.sum())]*int(good.sum()))
    x=np.vstack(xx); y=np.vstack(yy); weights=np.asarray(weights); weights/=weights.sum()
    if variant in ['rotation','rotation_gain']:
        u,_,vt=np.linalg.svd(x.T@(weights[:,None]*y))
        fix=np.eye(3);fix[-1,-1]=np.linalg.det(u@vt)
        coef=u@fix@vt
        if variant=='rotation_gain': coef*=np.sum(weights[:,None]*(x@coef)*y)/np.sum(weights[:,None]*x*x)
        return dict(coef=coef.tolist(), scale=[1]*3, variant=variant, branch=branch, lag=lag)
    scale=np.maximum(np.sqrt(np.sum(weights[:,None]*x*x,axis=0)), .05)
    z=x/scale
    # Fixed predeclared ridge (weighted MSE + .01 ||coefficients||²), no test tuning.
    if variant=='diagonal':
        co=np.diag(np.sum(weights[:,None]*z*y,axis=0)/(np.sum(weights[:,None]*z*z,axis=0)+.01))
    else:
        co=np.linalg.solve(z.T@(weights[:,None]*z)+.01*np.eye(z.shape[1]), z.T@(weights[:,None]*y))
    return dict(coef=co.tolist(),scale=scale.tolist(),variant=variant,branch=branch,lag=lag,
                input_singular_values=np.linalg.svd(z*np.sqrt(weights[:,None]),compute_uv=False).tolist())


def recover(tau,j, damping=.01):
    """Solve J.T W ~= tau by fixed-damping least squares; wrench=[F,M]."""
    return np.linalg.solve(
        j @ np.swapaxes(j, 1, 2) + damping**2*np.eye(6),
        np.einsum('nij,nj->ni', j, tau))


def predict(model,d):
    branch=model['branch']; x=d['f'] if branch=='force' else d['tau']
    if model['variant']=='pose': x=np.column_stack((x,d['dq']))
    out=(x/np.asarray(model['scale']))@np.asarray(model['coef'])
    j=d['j']; ref=d['w']; good=np.ones(len(x),dtype=bool)
    if model['lag']:
        ref,good=interp(d['t']-model['lag'],d['raw_series'])
        q,qgood=interp(d['t']-model['lag'],d['joints']);good &=qgood & np.isfinite(ref).all(axis=1)
        j=np.array([LocalIKSolver().compute_jacobian(v) for v in q])
    return (out if branch=='force' else recover(out,j)[:,:3]), ref[:,:3], good, out


def evaluate(trials, train_ids, test_ids, branch, variant, lag=0.):
    model=fit_model([trials[i] for i in train_ids],branch,variant,lag)
    results=[]
    for i in test_ids:
        d=trials[i]; pred,ref,good,rawpred=predict(model,d)
        moving=good & (d['speed']>.005)
        result=dict(label=d['meta']['label'],trial=d['meta']['trial'],mode=d['meta']['mode'],
                    branch=branch,variant=variant,lag=lag,
                    metrics=metrics(pred[good],ref[good]),moving_metrics=metrics(pred[moving],ref[moving]))
        if branch=='torque':
            if not lag:
                result['torque_rmse_axes_nm']=np.sqrt(np.mean((rawpred[good]-d['target_tau'][good])**2,axis=0)).tolist()
        results.append((result,pred,ref,good))
    return model,results


def inventory():
    out=[]
    for p in sorted((ROOT/'cocarry_logs').glob('hc_force*/**/events.jsonl')):
        if '01_static_cog' in str(p): continue
        counts=Counter();running=0;args={};faults=set()
        for line in p.open():
            e=json.loads(line)
            if e['kind']=='start':args=e.get('arguments',{})
            if e['kind']=='register' and e.get('response',{}).get('success'):counts[e['address']]+=1
            if e['kind']=='topic' and e['topic']=='/cocarry/status':
                s=e['message']['data'];running+=s.startswith('RUNNING')
                if s.startswith('FAULT'):faults.add(s)
        out.append(dict(path=str(p.parent.relative_to(ROOT)),arguments=args,register_counts=dict(counts),
                        running_status_messages=running,faults=sorted(faults),
                        selected=p.parent.name in {t[0] for t in TRIALS}))
    return out


def write_report(output):
    """Rebuild human-readable report solely from saved immutable analysis outputs."""
    quality=json.loads((output/'data_quality.json').read_text())
    result=json.loads((output/'results.json').read_text())
    rows=result['results']; models=json.loads((output/'candidate_models.json').read_text())['models']
    z=np.load(output/'paired_samples.npz',allow_pickle=False)
    trials=[]
    for i,meta in enumerate(quality):
        d={k:z[f'{i}_{k}'] for k in ['t','f','q','dq','w','j','speed','phase']}
        d['tau']=z[f'{i}_tau'] if f'{i}_tau' in z else None
        d['target_tau']=np.einsum('nji,nj->ni',d['j'],d['w']);d['meta']=meta;trials.append(d)
    pooled={}
    for branch in ['force','torque']:
        pp=[];rr=[];speed=[]
        for i,d in enumerate(trials[:6]):
            pred,ref,valid,_=predict(models[f'loto_{branch}_linear_{i}'],d)
            pp.append(pred[valid]);rr.append(ref[valid]);speed.extend(d['speed'][valid])
        pred=np.vstack(pp);ref=np.vstack(rr);moving=np.asarray(speed)>.005
        pooled[branch]=dict(all_running=metrics(pred,ref),moving=metrics(pred[moving],ref[moving]))
    (output/'pooled_metrics.json').write_text(json.dumps(pooled,indent=2)+'\n')
    # Full-training candidates retained separately; test metrics refer to excluded-trial models.
    candidates={branch:fit_model(trials[:6],branch,'linear') for branch in ['force','torque']}
    (output/'full_GT_candidates.json').write_text(json.dumps(dict(
        status='EXPLORATORY_NOT_COMPLETE_CALIBRATION_NOT_FOR_CONTROL',models=candidates,
        formulas={'row_vector':'y = (x / scale) @ coef',
                  'force_input':'decoded M320:322 minus initial no-contact per-trial baseline',
                  'torque_input':'decoded M310:315 minus initial no-contact per-trial baseline',
                  'torque_target':'J(q).T @ [Axia gravity/bias-corrected force, baseline-corrected moment shifted to flange]',
                  'force_recovery':'recover(predicted_torque, J(q)), damping .01, characteristic length .3m'},
        limitations=['No M310:315 in GRU/MJM logs','Axia inertial wrench not fully removed',
                     'Requires fresh initial no-contact baseline','Physical Axia axes assumed for supplied offset']),indent=2)+'\n')
    def get(protocol,label,branch,variant='linear'):
        return next(r for r in rows if r['protocol']==protocol and r['label']==label and r['branch']==branch and r['variant']==variant)
    def f(v):return '—' if v is None else f'{v:.2f}'
    lines=['# So sánh calibration động HC: M320–322 và M310–315', '',
        'Ngày phân tích: 17/09/2026. Trạng thái: **offline, chưa calibration hoàn chỉnh, chưa triển khai điều khiển**.', '',
        '## Kết luận', '',
        '**Nhánh torque + Jacobian có triển vọng rõ hơn nhánh lực trực tiếp trên dữ liệu Ground Truth được giữ ngoài tập fit. '
        'Chưa thể xác nhận toàn tuyến hoặc GRU/GRU+MJM vì các log AI không có M310–315.**', '',
        'Không dùng sai số fit trên chính dữ liệu học để kết luận. Các con số dưới đây dùng toàn lượt ngoài tập học, '
        'hoặc toàn vùng Home/Target 1 ngoài tập học. Mô hình 6×6 là hiệu chỉnh thực nghiệm, không chứng minh đã nhận dạng đúng '
        'toàn bộ tham số vật lý của cảm biến.', '',
        '## Dữ liệu và hình học', '',
        '- 6 lượt Ground Truth chính đã được tài liệu tiến độ giữ lại: Home X/Y/Z và Target 1 X/Y/Z. '
        'Giữ quy tắc loại Target 1 Z lượt 3 và mọi dữ liệu sau timeout. Không đưa trial lỗi bị thay thế vào fit.',
        '- 4 lượt AI: 16:43 GRU T1, 16:59 GRU T2 trước fault, 17:37 và 17:46 GRU+MJM. '
        'Mỗi lượt chỉ có M320–325. Không thay M310–315 bằng joint_states.effort hay suy ngược từ M320 để tạo kiểm chứng giả.',
        '- Dùng trạng thái controller RUNNING kết hợp /run_status. Baseline cuối, đoạn sau Stop/Fault không được tính là chuyển động thành công.',
        '- Người dùng cung cấp Axia→flange=(0,-120,0) mm và Axia→tâm thanh=(-160,0,0) mm. '
        'Phân tích dùng flange→Axia=(0,+120,0) mm trong trục đo Axia; bỏ qua tấm đệm. '
        'Tâm thanh không được đồng nhất với CoG toàn cụm. Không sửa URDF/TF/control.',
        '- TF ghi trong log có tool0→Axia translation=0. Hình học 120 mm chỉ được áp dụng trong phân tích offline. '
        'Nếu các tọa độ người dùng nêu thuộc frame khác trục đo Axia, cần đổi biểu diễn trước khi dùng estimator thật.', '',
        '| Lượt | Có torque? | Mẫu RUNNING ghép được | Mẫu đang di chuyển >5 mm/s | Chu kỳ mẫu trung vị (s) |',
        '|---|---|---:|---:|---:|']
    for d in quality:lines.append(f"| {d['label']} | {'Có' if d['has_tau'] else 'Không'} | {d['paired_running_samples']} | {d['moving_samples']} | {f(d['sample_interval_s']['median'])} |")
    lines += ['', '## Phương pháp tái lập', '',
        '1. Giải mã (raw−10000)×0,1. Dùng trung điểm thời gian gửi/nhận từng register như xấp xỉ thời điểm đọc, '
        'không coi đó là timestamp lấy mẫu nội bộ controller. Nội suy từng kênh về timestamp M321 của scan hoàn chỉnh; '
        'không ngoại suy, không đi qua khoảng thiếu register >1,5 s hoặc Axia/joint >0,2 s. Đây là tái dựng offline, không làm tăng băng thông thật.',
        '2. Axia force: trừ gravity theo TF/FK với tải 1,126 kg và bias sensor từ baseline đầu lượt. '
        'Moment: trừ baseline đầu lượt rồi đổi frame và cộng r×F về flange/tool0. '
        'Giữ correction yaw −90° của source, không fit lại góc gá. Không dùng baseline cuối để sửa kết quả.',
        '3. Axia được làm trơn cửa sổ đối xứng 0,10 s cho phân tích. '
        'Tham chiếu vẫn có thể chứa quán tính tải: chưa phải wrench tiếp xúc đã bù động lực học hoàn chỉnh. '
        'CoG/inertia chưa được giả định từ tâm thanh. Các trial giữ orientation gần như cố định nên biến thiên moment trọng lực rất nhỏ.',
        '4. Nhánh lực thử rotation đúng SO(3), rotation+gain chung, ma trận 3×3, và ma trận kèm q−q0. '
        'Nhánh torque thử gain từng khớp, ma trận 6×6, và ma trận kèm q−q0. '
        'Target torque là J(q)^T W_Axia; input là M310–315 đã trừ baseline đầu. '
        'Tại test chỉ dùng register, q và baseline đầu để dự đoán; không đưa Axia test vào công thức dự đoán.',
        '5. Ridge cố định 0,01 trên MSE có trọng số cân bằng giữa các trial, chuẩn hóa input bằng RMS training. '
        'Không chọn regularization bằng test. Ma trận tùy ý có thể hấp thụ sai lệch frame/gain/baseline và không được gọi là ma trận xoay.',
        '6. Khôi phục wrench bằng DLS của Jacobian với chiều dài chuẩn 0,3 m và damping=0,01. '
        'Kiểm tra riêng vòng W→J^TW→W để phân biệt lỗi nghịch đảo với lỗi tín hiệu.',
        '7. Góc chỉ tính khi norm cả hai lực ≥4 N. Luôn báo số mẫu đủ điều kiện để tránh chọn lọc kết quả đẹp. '
        'RMSE dùng cả lực nhỏ. Ngưỡng 4 N phục vụ báo cáo, chưa phải ngưỡng conflict.', '',
        '## So sánh chính: giữ nguyên một lượt GT ngoài tập fit', '',
        'Bảng ưu tiên **chỉ đoạn có tốc độ EE >0,005 m/s**. Nguồn vận tốc: gradient FK theo timestamp mẫu ghép; '
        'đây là tiêu chí phân tích, không phải thay đổi giới hạn controller.', '',
        '| Lượt test | Số mẫu động | RMSE nhánh lực 3×3 (N) | RMSE nhánh torque 6×6 + J (N) |',
        '|---|---:|---:|---:|']
    for d in quality[:6]:
        rf=get('GT_leave_trial_out',d['label'],'force')['moving_metrics']
        rt=get('GT_leave_trial_out',d['label'],'torque')['moving_metrics']
        lines.append(f"| {d['label']} | {rf['n']} | {f(rf.get('rmse_vector_n'))} | {f(rt.get('rmse_vector_n'))} |")
    lines += [f"| **Gộp theo mẫu** | **{pooled['force']['moving']['n']}** | **{f(pooled['force']['moving']['rmse_vector_n'])}** | **{f(pooled['torque']['moving']['rmse_vector_n'])}** |", '',
        'Nếu tính toàn RUNNING gồm cả đoạn nhả lực/đứng yên: RMSE gộp của nhánh lực = '
        f"{f(pooled['force']['all_running']['rmse_vector_n'])} N, nhánh torque = {f(pooled['torque']['all_running']['rmse_vector_n'])} N. "
        'Không dùng con số này thay cho sai số khi chuyển động.', '',
        '| Lượt | Góc P95 nhánh lực (°) | Số góc / số Axia ≥4N | Góc P95 nhánh torque (°) | Số góc / số Axia ≥4N |',
        '|---|---:|---:|---:|---:|']
    for d in quality[:6]:
        ms=[get('GT_leave_trial_out',d['label'],b)['metrics'] for b in ['force','torque']]
        entries=[f"{f((m['angle_deg'] or {}).get('p95'))} | {m['angle_n']}/{m['reference_ge4']}" for m in ms]
        lines.append('| '+d['label']+' | '+' | '.join(entries)+' |')
    lines += ['', 'Trong 6 lượt, rotation thuần chưa giải quyết được sai lệch: sai số động cao và một số hướng sai đáng kể. '
        'Nhánh torque cải thiện rõ nhưng vẫn có góc P95 lớn; chưa thể tuyên bố phù hợp mọi ngưỡng conflict.', '',
        '## Giữ toàn vùng ngoài tập học', '',
        '| Test | Train | RMSE động nhánh lực (N) | RMSE động nhánh torque (N) |',
        '|---|---|---:|---:|']
    for d in quality[:6]:
        rf=get('GT_leave_region_out',d['label'],'force')['moving_metrics'];rt=get('GT_leave_region_out',d['label'],'torque')['moving_metrics']
        lines.append(f"| {d['label']} | {'Target 1' if d['label'].startswith('Home') else 'Home'} | {f(rf.get('rmse_vector_n'))} | {f(rt.get('rmse_vector_n'))} |")
    lines += ['', '## GRU/GRU+MJM: chỉ kiểm chứng được nhánh lực', '',
        '| Test | Train GT, 3×3: RMSE động (N) | Train các lượt còn lại, 3×3+q: RMSE động (N) | Góc được tính / Axia ≥4N, mô hình sau |',
        '|---|---:|---:|---:|']
    for d in quality[6:]:
        first=get('GT_to_GRU_MJM',d['label'],'force')
        second=get('all_modes_leave_trial_out',d['label'],'force','pose');m=second['metrics']
        lines.append(f"| {d['label']} | {f(first['moving_metrics'].get('rmse_vector_n'))} | {f(second['moving_metrics'].get('rmse_vector_n'))} | {m['angle_n']}/{m['reference_ge4']} |")
    lines += ['', 'Bổ sung quỹ đạo AI vào training có cải thiện nhánh lực, nhưng độ phủ góc còn thiếu và các lượt cùng tuyến T2 '
        'có tương quan. Đây chưa phải kiểm chứng trên toàn workspace hoặc một session độc lập. '
        'Các trial GRU+MJM gồm cả FOLLOWER/LEADER; logger này không có nhãn role đồng bộ nên bảng không tách riêng LEADER MJM.', '',
        '## Độ lặp lại, trễ và hình học', '',
        '- Kiểm chứng giữ lần tác động cuối trong cùng trial cho nhánh torque tốt hơn kiểm chứng đổi lượt/vùng. '
        'Điều này cho thấy hiệu chỉnh cục bộ có ích, nhưng không được dùng kết quả cục bộ để thay cho khả năng tổng quát hóa.',
        '- Baseline Axia đầu lượt có scatter vector khoảng '
        f"{min(d['baseline_axia_vector_scatter_n'] for d in quality):.3f}–{max(d['baseline_axia_vector_scatter_n'] for d in quality):.3f} N "
        'trong các cửa sổ không tiếp xúc đã chọn. Đây chỉ là nhiễu ngắn hạn trong log, không phải độ chính xác tuyệt đối của Axia. '
        'Sau chuyển động có baseline lệch: bảng data_quality.json giữ mean/std và delta q để phân biệt trở về Home với dừng ở pose khác.',
        '- Quét lag [−0,2; 0; 0,2; 0,4; 0,6] s bằng validation lồng nhau trên trial training. '
        'Lag được chọn không nhất quán giữa lượt; chưa xác nhận một hằng số trễ dùng chung. '
        'Kết quả lag so với wrench ở thời điểm dịch chuyển, không phải ước lượng lực hiện tại. Lag âm cần tương lai và chỉ dùng chẩn đoán.',
        '- Thử lại nhánh torque với offset 0/102/138 mm và refit riêng. Sai khác force nhỏ trong bộ dữ liệu này; '
        'điều đó không xác nhận vị trí sensor không quan trọng: orientation gần cố định và ma trận 6×6 có thể hấp thụ một phần khác biệt. '
        'Hình học đúng vẫn cần thiết cho moment và tổng quát hóa.',
        '- Axia quán tính tịnh tiến được ước lượng sơ bộ từ đạo hàm FK làm chỉ báo độ lớn, chưa dùng để bù chính thức. '
        'Không xác định inertia toàn cụm hoặc đáp ứng lọc nội bộ HC từ các phép fit này.', '',
        '## Quyết định và bước tiếp theo cụ thể', '',
        '1. Giữ nhánh M310–315 + Jacobian làm ứng viên ưu tiên; kết quả hiện tại hỗ trợ tiếp tục theo nhánh này, chưa hỗ trợ triển khai hoàn chỉnh.',
        '2. Phần thiếu tối thiểu là dữ liệu M310–315 trên tuyến GRU/GRU+MJM. '
        'Nên ghi đồng thời nhóm torque+wrench với timestamp, và cả nhãn role; không cần thu lại toàn bộ thí nghiệm tĩnh.',
        '3. Hiện nhóm all chỉ khoảng 1,3 Hz; nội suy không khôi phục được đỉnh lực nhanh. '
        'Cần đánh giá đường lấy 12 register cần thiết hoặc publisher trong controller trước khi kết luận giới hạn độ trễ của estimator.',
        '4. Chốt tiêu chí sai số lực/góc, độ phủ và độ trễ theo ứng dụng; kiểm chứng session/lượt độc lập. '
        'Việc calibration ngoại lực thành công vẫn chưa tự kiểm chứng logic conflict/ý định robot.', '',
        '## Artifacts và kiểm tra', '',
        '- `inventory.json`: toàn bộ trial được rà soát và register thực sự đã ghi.',
        '- `data_quality.json`: baseline, timestamp, hình học, FK/TF và hash log nguồn.',
        '- `paired_samples.npz`: mẫu sau ghép để tái lập kiểm tra.',
        '- `results.json`, `metrics.csv`, `pooled_metrics.json`: các giao thức kiểm chứng và độ phủ.',
        '- `candidate_models.json`: mô hình của từng fold. `full_GT_candidates.json`: ứng viên fit toàn GT, ghi rõ chưa calibrated.',
        '- `trial_0.png`…`trial_9.png`: đồ thị dự đoán trên lượt chưa học.',
        '- Script: `analyze_hc_dynamic_branches.py`; kiểm thử số học trong `tests/test_hc_dynamic_branches.py`.',
        '- `py_compile` đạt; 4 kiểm thử số học đạt. Pytest chạy với `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` '
        'do plugin ROS launch_testing cài sẵn không tương thích pytest. Có cảnh báo phiên bản SciPy/NumPy; các phép kiểm tra số học đã chạy đạt.', '',
        'Bản `20260917_dynamic_branches_v1` là phép thử ban đầu với offset TF=0, trước khi người dùng cung cấp hình học. '
        '**Dùng bản v2_geometry này cho kết luận hiện tại.**', '',
        'Lệnh tái lập (phải chọn thư mục output mới để không ghi đè):', '', '```bash',
        'OPENBLAS_NUM_THREADS=1 python3 scripts/analyze_hc_dynamic_branches.py --output <thu_muc_moi>', '```', '']
    (output/'report_vi.md').write_text('\n'.join(lines))


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--skip-inventory',action='store_true');p.add_argument('--report-only',action='store_true');a=p.parse_args()
    if a.report_only:
        write_report(a.output);return
    a.output.mkdir(parents=True,exist_ok=False)
    if not a.skip_inventory:
        (a.output/'inventory.json').write_text(json.dumps(inventory(),indent=2)+'\n')
    paths={p.parent.name:p.parent for p in (ROOT/'cocarry_logs').glob('hc_force*/**/events.jsonl')}
    trials=[parse_trial(paths[name],mode,label) for name,mode,label in TRIALS]
    (a.output/'data_quality.json').write_text(json.dumps([d['meta'] for d in trials],indent=2)+'\n')
    np.savez_compressed(a.output/'paired_samples.npz', **{f'{i}_{k}':d[k] for i,d in enumerate(trials)
                        for k in ['t','f','tau','q','dq','w','j','speed','phase'] if d[k] is not None})
    gt=list(range(6)); future=list(range(6,10)); results=[];models={};predictions={}
    variants={'force':['rotation','rotation_gain','linear','pose'], 'torque':['diagonal','linear','pose']}
    for branch,options in variants.items():
        for variant in options:
            print('LOTO',branch,variant,flush=True)
            for test in gt:
                model,res=evaluate(trials,[i for i in gt if i!=test],[test],branch,variant)
                r,pred,ref,good=res[0];r['protocol']='GT_leave_trial_out';results.append(r)
                models[f'loto_{branch}_{variant}_{test}']=model
                predictions[(test,branch,variant)]=pred
            if branch=='force':
                model,res=evaluate(trials,gt,future,branch,variant)
                models[f'GT_to_AI_{variant}']=model
                for r,pred,ref,good in res:
                    r['protocol']='GT_to_GRU_MJM';results.append(r)
                    idx=next(i for i,d in enumerate(trials) if d['meta']['trial']==r['trial'])
                    predictions[(idx,branch,variant)]=pred
    # Harder region holdout: no dynamic trial from the test region is in fitting.
    for branch in ['force','torque']:
        for variant in ['linear','pose']:
            for train,test in [([0,1,2],[3,4,5]),([3,4,5],[0,1,2])]:
                _,res=evaluate(trials,train,test,branch,variant)
                for r,_,_,_ in res:r['protocol']='GT_leave_region_out';results.append(r)
    # Can adding dynamic AI trajectories improve transfer? Still exclude whole test trial.
    for variant in ['linear','pose']:
        for test in range(10):
            model,res=evaluate(trials,[i for i in range(10) if i!=test],[test],'force',variant)
            r,_,_,_=res[0];r['protocol']='all_modes_leave_trial_out';results.append(r)
            models[f'all_modes_loto_force_{variant}_{test}']=model
    # Nested lag diagnostic: select lag using training trials only, linear model fixed.
    # Evaluates delayed wrench, not a real-time current-force claim.
    lag_grid=[-.2,0.,.2,.4,.6]
    lag_choices=[]
    for branch in ['force','torque']:
        for test in gt:
            train=[i for i in gt if i!=test]; scores=[]
            for lag in lag_grid:
                score=[]
                for val in train:
                    _,res=evaluate(trials,[i for i in train if i!=val],[val],branch,'linear',lag)
                    score.append(res[0][0]['metrics']['rmse_vector_n'])
                scores.append(float(np.mean(score)))
            lag=lag_grid[int(np.argmin(scores))]
            model,res=evaluate(trials,train,[test],branch,'linear',lag)
            r,_,_,_=res[0];r['protocol']='nested_lag_delayed_reference';results.append(r)
            lag_choices.append(dict(branch=branch,test=trials[test]['meta']['label'],selected_lag=lag,grid=lag_grid,inner_rmse=scores))
    # Within-trial block validation: complete last action repetition held out, never random rows.
    for i,d in enumerate(trials[:6]):
        import re
        reps=np.array([int(m.group(1)) if (m:=re.search(r'_([123])$',ph)) else 0 for ph in d['phase']])
        last=reps.max();test=reps==last;train=(reps>0)&(reps<last)
        if train.sum()<10 or test.sum()<10: continue
        def subset(mask):
            z=d.copy()
            for k in ['t','f','tau','q','dq','w','target_tau','j','speed','phase','tool_rot','sensor_rot']:z[k]=d[k][mask]
            return z
        ds=[subset(train),subset(test)]
        for branch in ['force','torque']:
            _,res=evaluate(ds,[0],[1],branch,'linear')
            r,_,_,_=res[0];r['protocol']='within_trial_last_repeat';results.append(r)
    # Offset sensitivity: same physics model is trained/tested separately for each assumed offset.
    sensitivities=[]
    for offset in [np.zeros(3),np.array([0,.102,0]),np.array([0,.138,0])]:
            ds=[]
            for d in trials[:6]:
                z=d.copy(); w=d['w'].copy()
                r=np.einsum('nij,j->ni',d['sensor_rot'],offset-FLANGE_TO_AXIA_SENSOR_M)
                w[:,3:]+=np.cross(r,w[:,:3]);z['w']=w
                z['target_tau']=np.einsum('nji,nj->ni',z['j'],w);ds.append(z)
            for test in gt:
                _,res=evaluate(ds,[i for i in gt if i!=test],[test],'torque','linear')
                r,_,_,_=res[0];r['assumed_flange_to_axia_sensor_axes_m']=offset.tolist();sensitivities.append(r)
    # How much force DLS loses even when fed exact Axia-derived joint torques.
    reconstruction=[dict(label=d['meta']['label'],metrics=metrics(recover(d['target_tau'],d['j'])[:,:3],d['w'][:,:3]))
                    for d in trials[:6]]
    controls=[dict(label=d['meta']['label'],zero_force=metrics(np.zeros_like(d['f']),d['w'][:,:3]),
                   uncorrected_force=metrics(d['f'],d['w'][:,:3]),
                   uncorrected_torque=metrics(recover(d['tau'],d['j'])[:,:3],d['w'][:,:3]) if d['tau'] is not None else None)
              for d in trials]
    (a.output/'results.json').write_text(json.dumps(dict(status='offline_comparison_not_deployed',results=results,
        lag_selection=lag_choices,offset_sensitivity=sensitivities,exact_target_torque_reconstruction=reconstruction,
        controls=controls,geometry=dict(flange_to_axia_in_sensor_axes_m=FLANGE_TO_AXIA_SENSOR_M.tolist(),
        spacers_included=False,axes_assumption='physical Axia measurement axes, existing source yaw correction')),
        indent=2)+'\n')
    (a.output/'candidate_models.json').write_text(json.dumps(dict(status='NOT_CALIBRATED_NOT_FOR_CONTROL',models=models),indent=2)+'\n')
    with (a.output/'metrics.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['protocol','label','branch','variant','lag_s','n','rmse_N','angle_n','angle_median_deg','angle_p95_deg','angle_coverage','moving_n','moving_rmse_N'])
        for r in results:
            m=r['metrics'];an=m['angle_deg'] or {};mm=r['moving_metrics']
            writer.writerow([r['protocol'],r['label'],r['branch'],r['variant'],r['lag'],m['n'],m['rmse_vector_n'],m['angle_n'],an.get('median'),an.get('p95'),m['angle_coverage'],mm['n'],mm.get('rmse_vector_n')])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for i,d in enumerate(trials):
        fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
        for k,ax in enumerate(axes):
            ax.plot(d['t'],d['w'][:,k],label='Axia: gravity + initial bias corrected',color='black',lw=1)
            ax.plot(d['t'],predictions[i,'force','linear'][:,k],label='M320-322 linear: held-out trial',lw=1)
            if i<6:ax.plot(d['t'],predictions[i,'torque','linear'][:,k],label='M310-315 linear + Jacobian: held-out trial',lw=1)
            ax.set_ylabel('F'+'xyz'[k]+' [N]');ax.grid(alpha=.2)
        axes[0].legend(fontsize=8);axes[-1].set_xlabel('Seconds since logger start')
        fig.suptitle(d['meta']['label']);fig.tight_layout();fig.savefig(a.output/f'trial_{i}.png',dpi=140);plt.close(fig)
    write_report(a.output)
    print('Saved',a.output,flush=True)


if __name__=='__main__':main()
