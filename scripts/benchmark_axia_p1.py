#!/usr/bin/env python3
"""Offline P1 benchmark: measured rest noise, causal replay, known-input tests.

Never imports ROS/UI or publishes. Existing logs/configuration are immutable.
Selection uses September 17 only; September 18 remains a historical holdout.
"""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import time

import numpy as np
from scipy.ndimage import median_filter, uniform_filter1d

import calibrate_hc_p0_unified as p0
from axia_filter_candidates import PRESETS, replay, radial_deadband

ROOT=Path(__file__).resolve().parent.parent
P0_INPUT=ROOT/'cocarry_logs/hc_force_calibration/20260919_p0_unified_inputs_v2'
POLICY=dict(selection_day='20260917',heldout_day='20260918',sample_rate_hz=100,
            noise_rms_ratio_max=.50,single_spike_residual_ratio_p95_max=.10,
            synthetic_t90_ms_max=80.,direction_p95_deg_max=10.,
            tie_break='smallest synthetic median t90, then measured rest noise ratio',
            note='Engineering screening criteria fixed before running benchmark, not robot safety limits.')


def save(path,value):
    p0.save(path,value)


def stats(x):
    return p0.base.stats(np.asarray(x,float))


def split(day):
    return 'historical_holdout' if day>='20260918' else 'validation' if day=='20260917' else 'development'


def contiguous(times,valid):
    ix=np.flatnonzero(valid)
    if not len(ix): return []
    cuts=np.flatnonzero((np.diff(ix)>1)|(np.diff(times[ix])>.05)|(np.diff(times[ix])<=0))+1
    return [a for a in np.split(ix,cuts) if len(a)>=100]


def prepare(output):
    """Extract native raw at ~100 Hz; never upsample the ~2 Hz P0 paired arrays."""
    manifest=json.loads((P0_INPUT/'manifest.json').read_text())
    bs=json.loads((P0_INPUT/'baselines.json').read_text())
    ds,_=p0.read_prepared(P0_INPUT)
    force={d['meta']['trial']:d for d in ds if d['meta']['date']>='20260915'}
    bs=[b for b in bs if b['trial'][:8]>='20260915']
    selected={b['trial'] for b in bs}|set(force)
    arrays={}; segments=[]; inventory=[]
    for row in manifest:
        if row['trial'] not in selected: continue
        path=ROOT/row['path']
        if p0.digest(path)!=row['source_sha256']: raise ValueError('Source changed: '+str(path))
        _,src=p0.load_log(path)
        raw=src['raw']; joints=src['joints']
        if len(raw)<100 or len(joints)<2: continue
        t=raw[:,0]; x=raw[:,1:4]
        q,valid=p0.interpolate(t,joints)
        for changes in src['health'].values(): valid &= p0.at(t,changes,False)
        valid &= np.isfinite(x).all(axis=1)
        inventory.append(dict(trial=row['trial'],path=row['path'],sha256=row['source_sha256'],
                              raw_samples=len(raw),interval_s=stats(np.diff(t))))
        windows=[]
        for b in bs:
            if b['trial']==row['trial']:
                lo,hi=b['window_s']
                if hi-lo>=5:
                    # End of stable segment; at most 12 seconds, first two are warmup.
                    windows.append(('rest',max(lo,hi-12),hi,b))
        d=force.get(row['trial'])
        if d is not None:
            ft=d['t'][d['force_mask']]
            # Up to 30 s from the first interaction, plus two seconds of prefix.
            # Exact source health and P0 force intervals remain separate evaluation masks.
            lo=max(t[0],ft[0]-2); hi=min(ft[-1],lo+32)
            b=next(b for b in bs if b['trial']==row['trial'] and b['name']=='baseline')
            windows.append(('interaction',lo,hi,b))
        for kind,lo,hi,b in windows:
            mask=valid & (t>=lo)&(t<=hi)
            for ix in contiguous(t,mask):
                if t[ix[-1]]-t[ix[0]]<4: continue
                use_t=t[ix]; use_x=x[ix]; use_q=q[ix]
                meta=dict(id=len(segments),trial=row['trial'],date=row['date'],split=split(row['date']),
                          mode=row['mode'],route=row['route'],kind=kind,window_s=[float(use_t[0]),float(use_t[-1])],
                          samples=len(ix),source_sha256=row['source_sha256'],source=row['path'],
                          baseline_phase=b['name'],first_2s_excluded=True)
                ref_rotation=np.eye(3); rotations=None; bias=np.median(use_x,axis=0)
                if kind=='interaction':
                    try:
                        fixed=p0.base.transform_chain(src['edges'],'axia_sensor_link','tool0')[:3,:3]@p0.CORRECTION
                    except ValueError: continue
                    rotations=np.array([p0.SOLVER.forward_kinematics(a)[:3,:3]@fixed for a in use_q])
                    ref_rotation=p0.SOLVER.forward_kinematics(b['q'])[:3,:3]@fixed
                    bias=np.array(b['raw'])[:3]-ref_rotation.T@p0.GRAVITY
                prefix=f'{len(segments)}_'
                arrays[prefix+'t']=use_t-use_t[0]; arrays[prefix+'raw']=use_x
                arrays[prefix+'bias']=bias
                if rotations is not None: arrays[prefix+'rotation']=rotations
                segments.append(meta)
        print('prepared',row['trial'],'segments',len(segments),flush=True)
    np.savez_compressed(output/'raw_segments.npz',**arrays)
    save(output/'segments.json',segments); save(output/'source_inventory.json',inventory)
    save(output/'preparation.json',dict(p0_manifest_sha256=p0.digest(P0_INPUT/'manifest.json'),
        script_sha256=p0.digest(Path(__file__)),segments=len(segments),
        counter=dict(Counter((m['split']+':'+m['kind']) for m in segments)),
        packet_raw_before_software_filter=True,hardware_tare_may_already_be_applied=True,
        assumptions=['Native raw topic, not inferred from human_force deadband.',
         'Measured rest windows inherit explicit no-contact and stationarity gates from P0.',
         'Only continuous healthy raw/joint sequences are evaluated; no interpolation over a gap >50 ms.',
         'Interaction subset is first 30 s after two-second prefix; not all trial dynamics.',
         'No true reference of human intent exists in recorded trials.',
         'Default/preset replay is counterfactual: actual historical UI alpha was not recorded.']))


def load_segments(prepared):
    z=np.load(prepared/'raw_segments.npz',allow_pickle=False)
    for m in json.loads((prepared/'segments.json').read_text()):
        i=m['id']; d={key.split('_',1)[1]:z[key] for key in z.files if key.startswith(f'{i}_')}
        d['meta']=m; yield d


def vector_metrics(pred,ref):
    good=np.isfinite(pred).all(axis=1)&np.isfinite(ref).all(axis=1)
    p=pred[good]; r=ref[good]; error=p-r
    rn=np.linalg.norm(r,axis=1); pn=np.linalg.norm(p,axis=1)
    eligible=rn>=4; angle_mask=eligible&(pn>=.5)
    angle=np.degrees(np.arccos(np.clip(np.sum(p[angle_mask]*r[angle_mask],axis=1)/(pn[angle_mask]*rn[angle_mask]),-1,1)))
    return dict(n=len(p),rmse_vector_n=float(np.sqrt(np.mean(np.sum(error*error,axis=1)))),
                p95_error_n=float(np.percentile(np.linalg.norm(error,axis=1),95)),angle_deg=stats(angle),
                angle_eligible=int(eligible.sum()),angle_count=int(angle_mask.sum()),
                low_output_fraction=float(np.mean(pn[eligible]<.5)) if eligible.any() else None)


def first_sustained(t,values,threshold,lo,hi,count=3):
    good=(t>=lo)&(t<hi)&(values>=threshold)
    for i in np.flatnonzero(good):
        if i+count<=len(t) and np.all(good[i:i+count]): return float(t[i])
    return None


def observed_onsets(t,reference,pred):
    r=np.linalg.norm(reference,axis=1); p=np.linalg.norm(pred,axis=1)
    candidates=np.flatnonzero((r[1:]>=4)&(r[:-1]<4))+1
    delays=[]; missed=0; last=-1.
    for i in candidates:
        if t[i]<2 or t[i]-last<.8: continue
        before=(t>=t[i]-.2)&(t<t[i])
        after=(t>=t[i])&(t<t[i]+.05)
        if before.sum()<5 or np.median(r[before])>=3.5 or not np.all(r[after]>=4): continue
        last=t[i]
        cross=first_sustained(t,p,4,t[i]-.2,t[i]+.5)
        if cross is None: missed+=1
        else: delays.append((cross-t[i])*1000)
    return dict(delay_ms=stats(delays),missed=missed)


def impulse_like_indices(raw,active):
    """Offline annotation only: isolated excursion with neighboring return.

    Future samples are allowed for this metric annotation, never in the filter.
    A real brief physical force can satisfy this rule; it is not a noise label.
    """
    center=median_filter(raw,size=(9,1),mode='nearest')
    mad=median_filter(np.abs(raw-center),size=(21,1),mode='nearest')
    threshold=np.maximum(.5,6*1.4826*np.linalg.norm(mad,axis=1))
    distance=np.linalg.norm(raw-center,axis=1)
    neighbors=np.full(len(raw),np.inf)
    neighbors[1:-1]=np.linalg.norm(raw[2:]-raw[:-2],axis=1)
    return np.flatnonzero(active & (distance>threshold) & (neighbors<.15)),center,distance


def real_benchmark(prepared,output):
    rows=[]; stages={}; bank={}
    for n,d in enumerate(load_segments(prepared)):
        t=d['t']; raw=d['raw']; meta=d['meta']; active=t>=2
        proxy=uniform_filter1d(median_filter(raw,size=(5,1),mode='nearest'),size=5,axis=0,mode='nearest')
        impulse_ix,local_center,impulse_amplitude=impulse_like_indices(raw,active)
        if meta['kind']=='rest':
            center=np.median(raw[active],axis=0)
            bank.setdefault(meta['split'],[]).append(raw[active]-center)
        else:
            reference=np.einsum('nij,nj->ni',d['rotation'],proxy-d['bias'])-p0.GRAVITY
        for name in PRESETS:
            began=time.perf_counter(); y,pre,rejected=replay(name,t,raw,stages=True)
            item=dict(segment=meta['id'],trial=meta['trial'],split=meta['split'],kind=meta['kind'],filter=name,
                      samples=int(active.sum()),compute_us_per_sample=(time.perf_counter()-began)*1e6/len(t),
                      rejected_samples=int(rejected[active].sum()))
            item['recorded_impulse_like_events']=len(impulse_ix)
            item['recorded_impulse_like_output_ratios']=stats([
                np.max(np.linalg.norm(y[i:min(i+10,len(t))]-local_center[i:min(i+10,len(t))],axis=1))/impulse_amplitude[i]
                for i in impulse_ix])
            if meta['kind']=='rest':
                residual=y[active]-center
                item.update(rms_n=float(np.sqrt(np.mean(np.sum(residual**2,axis=1)))),
                            p95_norm_n=float(np.percentile(np.linalg.norm(residual,axis=1),95)))
            else:
                base_y=np.einsum('nij,nj->ni',d['rotation'],y-d['bias'])-p0.GRAVITY
                item.update(proxy_metrics=vector_metrics(base_y[active],reference[active]),
                            proxy_onset=observed_onsets(t,reference,base_y))
            rows.append(item)
            # Save all stages for representative first segment of each split/type.
            key=meta['split']+'_'+meta['kind']
            if key+'_segment' not in stages or int(stages[key+'_segment'])==meta['id']:
                stages[key+'_segment']=np.array(meta['id']); stages[key+'_t']=t; stages[key+'_raw']=raw
                stages[key+'_'+name+'_despiked']=pre; stages[key+'_'+name+'_filtered']=y
                if meta['kind']=='interaction':
                    stages[key+'_'+name+'_pre_deadband']=base_y
                    stages[key+'_'+name+'_post_deadband']=radial_deadband(base_y)
                    stages[key+'_reference_proxy']=reference
        print('replayed',n+1,meta['kind'],meta['trial'],flush=True)
    save(output/'real_metrics.json',rows)
    np.savez_compressed(output/'stage_replays.npz',**stages)
    return rows,{k:np.vstack(v) for k,v in bank.items()}


def synthetic_benchmark(bank,output):
    """Known signals + measured noise. Synthetic force is not a physical validation."""
    t=np.arange(0,11,.01); rows=[]; examples={}
    directions=[np.array(a,float)/np.linalg.norm(a) for a in [[1,0,0],[0,1,0],[0,0,1],[1,1,1]]]
    for split_name,noise_bank in sorted(bank.items()):
        rng=np.random.default_rng({'development':601,'validation':701,'historical_holdout':801}[split_name])
        for case in range(6):
            truth=np.zeros((len(t),3)); events=[]
            if case<4:
                axis=directions[case]
                truth[(t>=2)&(t<5)]=axis*8; truth[(t>=5)&(t<8)]=-axis*8
                events=[(2.,np.zeros(3),axis*8),(5.,axis*8,-axis*8),(8.,-axis*8,np.zeros(3))]
            elif case==4:
                amp=8*np.clip((t-2)/1.5,0,1)*np.clip((9-t)/1.5,0,1)
                truth=amp[:,None]*directions[3]
            else:
                # Smooth 3D direction change with known nonzero norm, 0.5 Hz.
                truth=np.column_stack((8*np.cos(np.pi*t),8*np.sin(np.pi*t),2*np.sin(.5*np.pi*t)))
            start=int(rng.integers(0,max(1,len(noise_bank)-len(t))))
            noise=noise_bank[np.arange(start,start+len(t))%len(noise_bank)]
            measured=truth+noise; contaminated=measured.copy(); impulses=[]
            for i,(ts,width) in enumerate([(1.,1),(3.5,1),(6.5,1),(9.5,1),(4.,2),(7.,3)]):
                index=int(round(ts/.01)); direction=rng.normal(size=3); direction/=np.linalg.norm(direction)
                amplitude=10. if i%2 else 20.
                contaminated[index:index+width]+=amplitude*direction
                impulses.append((index,width,amplitude))
            for name in PRESETS:
                y=replay(name,t,measured); spike_y=replay(name,t,contaminated)
                t90=[]; rises=[]; delays=[]; failures=0
                for onset,old,new in events:
                    delta=new-old; progress=(y-old)@delta/np.dot(delta,delta)
                    first=first_sustained(t,progress,.1,onset,onset+1)
                    last=first_sustained(t,progress,.9,onset,onset+1)
                    if first is None or last is None: failures+=1
                    else:
                        rises.append((last-first)*1000); t90.append((last-onset)*1000)
                    # Physical 4 N intention crossing only for zero->force events.
                    if np.linalg.norm(old)<1e-9:
                        crossing=first_sustained(t,np.linalg.norm(y,axis=1),4,onset,onset+1)
                        if crossing is not None: delays.append((crossing-onset)*1000)
                peaks={1:[],2:[],3:[]}
                for index,width,amplitude in impulses:
                    residual=np.linalg.norm(spike_y[index:index+40]-y[index:index+40],axis=1)
                    peaks[width].append(float(np.max(residual)/amplitude))
                m=vector_metrics(y[t>=2],truth[t>=2])
                rows.append(dict(split=split_name,case=case,filter=name,metrics=m,
                     t90_ms=t90,rise_10_90_ms=rises,intention_delay_4n_ms=delays,step_failures=failures,
                     spike_peak_ratios={str(k):v for k,v in peaks.items()}))
                if split_name=='historical_holdout' and case in [0,5]:
                    key=f'case{case}'; examples[key+'_t']=t; examples[key+'_truth']=truth
                    examples[key+'_input']=measured; examples[key+'_contaminated']=contaminated
                    examples[key+'_'+name]=y; examples[key+'_'+name+'_spikes']=spike_y
            print('synthetic',split_name,'case',case,flush=True)
    save(output/'synthetic_metrics.json',rows)
    np.savez_compressed(output/'synthetic_examples.npz',**examples)
    return rows


def summarize(real,synthetic):
    rows=[]
    for split_name in ['development','validation','historical_holdout']:
        raw={r['segment']:r for r in real if r['split']==split_name and r['kind']=='rest' and r['filter']=='raw'}
        for name in PRESETS:
            rest=[r for r in real if r['split']==split_name and r['kind']=='rest' and r['filter']==name]
            synth=[r for r in synthetic if r['split']==split_name and r['filter']==name]
            if not rest or not synth: continue
            ratio=float(np.median([r['rms_n']/max(raw[r['segment']]['rms_n'],1e-12) for r in rest]))
            t90=[v for r in synth for v in r['t90_ms']]
            rise=[v for r in synth for v in r['rise_10_90_ms']]
            single=[v for r in synth for v in r['spike_peak_ratios']['1']]
            smooth=[r for r in synth if r['case']==5][0]['metrics']
            row=dict(split=split_name,filter=name,rest_segments=len(rest),noise_rms_ratio=ratio,
                rest_rms_median_n=float(np.median([r['rms_n'] for r in rest])),
                rest_p95_median_n=float(np.median([r['p95_norm_n'] for r in rest])),
                t90_median_ms=float(np.median(t90)) if t90 else None,
                t90_max_ms=float(np.max(t90)) if t90 else None,
                rise_10_90_median_ms=float(np.median(rise)) if rise else None,
                intention_delay_4n_ms=stats([v for r in synth for v in r['intention_delay_4n_ms']]),
                single_spike_ratio_p95=float(np.percentile(single,95)),
                single_spike_survival_fraction=float(np.mean(np.asarray(single)>.1)),
                double_spike_ratio_p95=float(np.percentile([v for r in synth for v in r['spike_peak_ratios']['2']],95)),
                triple_spike_ratio_p95=float(np.percentile([v for r in synth for v in r['spike_peak_ratios']['3']],95)),
                smooth_direction_p95_deg=smooth['angle_deg']['p95'] if smooth['angle_deg'] else None,
                step_failures=sum(r['step_failures'] for r in synth))
            row['passes_screen']=bool(ratio<=POLICY['noise_rms_ratio_max'] and
                row['single_spike_ratio_p95']<=POLICY['single_spike_residual_ratio_p95_max'] and
                row['t90_max_ms'] is not None and row['t90_max_ms']<=POLICY['synthetic_t90_ms_max']+1e-6 and
                row['smooth_direction_p95_deg'] is not None and row['smooth_direction_p95_deg']<=POLICY['direction_p95_deg_max'] and
                row['step_failures']==0)
            rows.append(row)
    return rows


def report(output,prepared,summary,selection):
    chosen=selection['selected']
    lines=['# P1 — Benchmark bộ lọc Axia offline, 19/09/2026','',
        f'Candidate đề xuất: **{chosen or "chưa có candidate vượt toàn bộ screen"}**. Chưa triển khai UI/ROS hoặc chạy shadow trực tiếp.', '',
        'Preset Raw hiện tại là median-5 với alpha=1.0; nó không phải packet raw. '
        'Preset median-5 + EMA 0.1 cũng được đối chứng chính xác với class ForceFilter trong source bằng test AST.', '',
        '## Protocol và selection', '',
        'Dùng raw trước lọc trong event logs. Các window không tiếp xúc có marker và kiểm tra khớp/health từ P0; '
        'mỗi window lấy tối đa 12 s và bỏ 2 s warmup. Interaction lấy tối đa 32 s gồm prefix, theo timestamp native.',
        'Ngày ≤16/09 là development; 17/09 là validation/chọn preset; 18/09 là historical holdout. '
        'Không dùng kết quả holdout để chỉnh preset. Không có force ground truth độc lập trong các thao tác thật.', '',
        'Screen định trước: noise RMS ratio ≤0.50 so với raw, single-spike residual P95 ≤10%, '
        't90 lớn nhất ≤80 ms, sai số hướng P95 ≤10° trên vector quay 0.5 Hz, không miss bước. '
        'Chọn t90 trung vị nhỏ nhất rồi noise ratio nhỏ nhất. Đây là tiêu chí sàng lọc kỹ thuật, không phải giới hạn an toàn robot.', '',
        'Tín hiệu tổng hợp dùng bước 8 N, đảo dấu, ramp, vector 3D quay; cộng noise lấy từ raw không tiếp xúc '
        'của đúng split. Xung chèn 10/20 N kéo dài 1/2/3 mẫu. Chỉ tín hiệu tổng hợp có đáp án lực/độ trễ đã biết.', '',
        '## Kết quả', '',
        '| Split | Filter | Noise/raw | RMS nghỉ (N) | t90 median/max (ms) | Rise 10–90 (ms) | Xung 1 mẫu P95 | Góc P95 (°) | Pass |',
        '|---|---|---:|---:|---:|---:|---:|---:|---|']
    for r in summary:
        lines.append(f"| {r['split']} | {r['filter']} | {r['noise_rms_ratio']:.3f} | {r['rest_rms_median_n']:.4f} | "
                     f"{r['t90_median_ms']:.1f}/{r['t90_max_ms']:.1f} | {r['rise_10_90_median_ms']:.1f} | "
                     f"{r['single_spike_ratio_p95']:.3f} | {r['smooth_direction_p95_deg']:.2f} | {r['passes_screen']} |")
    lines+=['','RMS nghỉ là RMS vector quanh cùng median raw của window, trước deadband. '
            'Không trừ riêng bias đầu ra của mỗi candidate để làm noise nhỏ giả. Noise ratio là median tỷ lệ theo window. '
            'Tỷ lệ xung đo peak chênh giữa output có/không xung trên cùng tín hiệu nền, chia amplitude chèn.', '',
            '## Thao tác thật và giới hạn', '',
            '`real_metrics.json` lưu RMS/P95 không tải và RMSE/góc so với proxy offline cho từng interaction. '
            'Proxy là centered median-5 rồi centered mean-5, chỉ dùng để chấm điểm, không chạy trong candidate. '
            'Proxy onset 4 N là chênh thời điểm theo proxy, không phải độ trễ ý định người thật. '
            'Không dùng proxy RMSE để chọn preset vì nó ưu ái bộ lọc giống chính proxy.', '',
            'Có thống kê thêm sự kiện giống xung đơn trong log: lệch median tâm 9 mẫu >max(0.5 N,6σ), '
            'hai mẫu lân cận trở lại trong 0.15 N. Đây chỉ là nhãn hậu kiểm có dùng mẫu tương lai, '
            'không khẳng định nhiễu điện hay đưa nhãn đó vào bộ lọc causal.', '',
            'Lọc trong sensor frame trước khi bù trọng lực/rotation giống pipeline hiện tại. Interaction dùng TF/FK, '
            'payload 1.126 kg và baseline có marker từ P0 để tái dựng base_link; không mặc định tái tạo đúng UI bias/tare lịch sử. '
            'Raw, despiked/median, filtered, trước và sau radial deadband 4 N được lưu tách riêng trong stage_replays.npz.', '',
            'Mọi bộ lọc có cửa sổ thời gian đều có thể làm mất xung lực vật lý rất ngắn. Bộ lọc causal không phân biệt '
            'chắc chắn một xung nhiễu với va chạm thật chỉ từ vài mẫu force. Xung 2/3 mẫu được báo riêng để lộ giới hạn; '
            'screen single-spike không phải chứng minh mọi burst được loại. Chưa dùng candidate cho force safety.', '',
            'Invalid/NaN/timestamp không tăng trả None và reset; gap >200 ms reset. Các metric xét đoạn liên tục, '
            'không suy force mới từ dữ liệu stale. Chính sách reset này là của replay, chưa thay watchdog hay reconnect UI.', '',
            '## Cấu hình và bước shadow', '',
            'Các preset cố định nằm trong axia_filter_candidates.py; thuật toán Hampel-confirm là biến thể riêng có '
            'xác nhận thay đổi bền vững sau hai mẫu, không phải Hampel chuẩn nguyên bản. Adaptive dùng robust scatter '
            'của sai phân và innovation để đổi cutoff 1.5–15 Hz, dùng chung gain cho ba trục. '
            'Low-pass bậc hai critically damped được đặt cutoff -3 dB 6 Hz.', '',
            'One Euro dựa trên cutoff tăng theo tốc độ; ở đây mở rộng vector dùng norm đạo hàm và cutoff chung, '
            'kèm median-3 trước lọc. [Mô tả thuật toán của tác giả](https://gery.casiez.net/1euro/).', '',
            'Chỉ sau review mới tích hợp candidate vào kênh shadow từ cùng packet Axia, giữ luồng điều khiển hiện tại. '
            'Shadow cần ghi timestamp, raw, các stage, state/reset và ngưỡng thực. Benchmark này không chứng minh '
            'candidate giảm sai số F_robot: cần đánh giá lại calibration trên cùng reference nếu sau này đổi bộ lọc.', '',
            '## Tái lập', '', '```bash',
            'OPENBLAS_NUM_THREADS=1 python3 scripts/benchmark_axia_p1.py --prepare-only --output <input_moi>',
            'OPENBLAS_NUM_THREADS=1 python3 scripts/benchmark_axia_p1.py --prepared <input_moi> --output <ket_qua_moi>',
            '```', '',f'Input đã đóng băng: `{prepared}`. Source hashes và snapshots nằm trong provenance.json/source_snapshot.json.']
    (output/'report_vi.md').write_text('\n'.join(lines)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    held=[r for r in summary if r['split']=='historical_holdout']
    fig,ax=plt.subplots(figsize=(9,5))
    for r in held:
        ax.scatter(r['t90_median_ms'],r['noise_rms_ratio'],s=60)
        ax.annotate(r['filter'],(r['t90_median_ms'],r['noise_rms_ratio']),xytext=(4,4),textcoords='offset points',fontsize=8)
    ax.set(xlabel='Known-step t90 median (ms)',ylabel='Measured rest RMS / raw RMS',title='Historical holdout: noise vs response delay')
    ax.grid(alpha=.2); fig.tight_layout(); fig.savefig(output/'noise_latency_tradeoff.png',dpi=160); plt.close(fig)
    z=np.load(output/'synthetic_examples.npz',allow_pickle=False)
    fig,axes=plt.subplots(2,1,figsize=(11,7))
    names=list(dict.fromkeys(['median5_raw','median5_ema01',chosen or 'median3_adaptive']))
    t=z['case0_t']; use=(t>1.85)&(t<2.5)
    axes[0].plot(t[use],z['case0_truth'][use,0],'k--',label='Known truth')
    for name in names: axes[0].plot(t[use],z['case0_'+name][use,0],label=name)
    axes[0].set(ylabel='Force X (N)',title='Synthetic step + measured holdout noise'); axes[0].legend()
    use=(t>.85)&(t<1.4)
    axes[1].plot(t[use],np.linalg.norm(z['case0_contaminated'][use],axis=1),color='.6',label='Input with injected single spike')
    for name in names: axes[1].plot(t[use],np.linalg.norm(z['case0_'+name+'_spikes'][use],axis=1),label=name)
    axes[1].set(xlabel='Time (s)',ylabel='Force norm (N)',title='Injected spike, not a recorded collision'); axes[1].legend()
    fig.tight_layout(); fig.savefig(output/'step_and_spike.png',dpi=160); plt.close(fig)


def run(prepared,output):
    for row in json.loads((prepared/'source_inventory.json').read_text()):
        if p0.digest(ROOT/row['path'])!=row['sha256']: raise ValueError('Source changed: '+row['path'])
    save(output/'benchmark_policy.json',dict(policy=POLICY,presets=PRESETS))
    real,bank=real_benchmark(prepared,output)
    synthetic=synthetic_benchmark(bank,output)
    summary=summarize(real,synthetic)
    candidates=[r for r in summary if r['split']=='validation' and r['passes_screen']]
    best=min(candidates,key=lambda r:(r['t90_median_ms'],r['noise_rms_ratio'])) if candidates else None
    selection=dict(selected=best['filter'] if best else None,policy=POLICY,
                   validation=best,status='OFFLINE_CANDIDATE_FOR_REVIEW_NOT_DEPLOYED',
                   holdout_used_for_selection=False,live_shadow_executed=False)
    save(output/'selection.json',selection); save(output/'summary.json',summary)
    if best: save(output/'candidate_filter.json',dict(name=best['filter'],config=PRESETS[best['filter']],
                  input='packet_native_sensor_force_n',gap_reset_sec=.2,control_enabled=False,shadow_only=True))
    with (output/'summary.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(summary[0])); writer.writeheader(); writer.writerows(summary)
    save(output/'provenance.json',dict(prepared=str(prepared),input_sha256=p0.digest(prepared/'raw_segments.npz'),
        benchmark_sha256=p0.digest(Path(__file__)),filter_sha256=p0.digest(ROOT/'axia_filter_candidates.py'),
        ui_sha256=p0.digest(ROOT/'axia_sensor_ui.py'),numpy=np.__version__,
        source_files=len(json.loads((prepared/'source_inventory.json').read_text())),
        live_pipeline_modified=False,hardware_access=False))
    save(output/'source_snapshot.json',dict(benchmark=Path(__file__).read_text(),
        filter=(ROOT/'axia_filter_candidates.py').read_text(),ui=(ROOT/'axia_sensor_ui.py').read_text()))
    report(output,prepared,summary,selection)
    print(json.dumps(selection,indent=2),flush=True)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--prepare-only',action='store_true')
    ap.add_argument('--prepared',type=Path)
    args=ap.parse_args(); args.output.mkdir(parents=True,exist_ok=False)
    if args.prepared: run(args.prepared.resolve(),args.output.resolve())
    else:
        prepare(args.output.resolve())
        if not args.prepare_only:
            out=args.output/'evaluation'; out.mkdir(); run(args.output.resolve(),out.resolve())


if __name__=='__main__':
    main()
