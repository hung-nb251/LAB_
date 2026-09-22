#!/usr/bin/env python3
"""So sanh cac luot co-carry theo muc radial deadband cua Axia.

Dung cho protocol A/B tai docs/axia_deadband_ab_protocol_20260921_vi.md.

Hai bay da gap khi phan tich ngay 21/09, script nay tranh san:

1. Loc theo ``control_phase`` lam cac dong khong con lien tuc theo thoi gian.
   Sai phan bac hai chay qua mot khoang trong se sinh gia toc gia rat lon. Vi
   vay moi dai luong vi phan chi duoc tinh RIENG trong tung doan lien tuc.
2. Cac luot co pha LEADER/MJM khong so sanh truc tiep duoc voi luot thuan
   FOLLOWER. Script bao cao ca hai cot: toan bo mau RUNNING va rieng tap con
   FOLLOWER_PREDICTION, de nguoi doc thay ro mask da bo di nhung gi.

Cach dung:
    python3 scripts/compare_axia_deadband_ab.py \
        --label 4.0N 20260922_0901 20260922_0906 20260922_0911 \
        --label 2.5N 20260922_0930 20260922_0935 20260922_0940
"""
import argparse
import glob
import os

import numpy as np
import pandas as pd

LOG_DIR = os.path.expanduser('~/cocarry_ws/cocarry_logs')
PREFIX = 'cocarry_admittance_3d_'
MIN_SEG = 45            # >= 3 s tai 15 Hz
MIN_RELEASE_SEC = 1.0


def contiguous(idx):
    if idx.size == 0:
        return []
    return np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)


def resolve(stem):
    hits = sorted(glob.glob(os.path.join(LOG_DIR, f'{PREFIX}*{stem}*.csv')))
    if not hits:
        raise FileNotFoundError(f'Khong tim thay log khop "{stem}" trong {LOG_DIR}')
    return hits[-1]


def metrics(df, mask, dt, lead_limit):
    """Tinh chi so tren cac doan lien tuc do ``mask`` chon ra."""
    segs = [s for s in contiguous(np.flatnonzero(mask)) if len(s) >= MIN_SEG]
    if not segs:
        return None
    ee = df[['actual_ee_x', 'actual_ee_y', 'actual_ee_z']].to_numpy()
    xr = df[['reference_xr', 'reference_yr', 'reference_zr']].to_numpy()
    fh = df[['f_human_x', 'f_human_y', 'f_human_z']].to_numpy()

    ref_acc, ee_acc, mags, leads = [], [], [], []
    reversals, duration = 0, 0.0
    drift, rel_rev, n_release = [], [], 0

    for s in segs:
        E, X, F = ee[s], xr[s], fh[s]
        mag = np.linalg.norm(F, axis=1)
        mags.append(mag)
        leads.append(np.linalg.norm(X - E, axis=1))
        ref_acc.append(np.linalg.norm(np.diff(X, 2, axis=0) / dt ** 2, axis=1))
        ee_acc.append(np.linalg.norm(np.diff(E, 2, axis=0) / dt ** 2, axis=1))
        reversals += count_reversals(E)
        duration += len(s) * dt
        for r in contiguous(np.flatnonzero(mag == 0.0)):
            if len(r) * dt < MIN_RELEASE_SEC:
                continue
            n_release += 1
            seg = E[r]
            drift.append(1000 * np.linalg.norm(seg - seg[0], axis=1).max())
            rel_rev.append(count_reversals(seg))

    mag = np.concatenate(mags)
    lead = np.concatenate(leads)
    nonzero = mag[mag > 0]
    return dict(
        n=int(sum(len(s) for s in segs)), nseg=len(segs), dur=duration,
        f_med=float(np.median(nonzero)) if nonzero.size else float('nan'),
        zero_pct=100.0 * float(np.mean(mag == 0.0)),
        sat_pct=100.0 * float(np.mean(lead > 0.95 * lead_limit)),
        ref_acc=float(np.concatenate(ref_acc).std()),
        ee_acc=float(np.concatenate(ee_acc).std()),
        rev_per_s=reversals / duration if duration else float('nan'),
        n_release=n_release,
        drift=float(np.mean(drift)) if drift else float('nan'),
        drift_max=float(np.max(drift)) if drift else float('nan'),
        rev_per_release=float(np.mean(rel_rev)) if rel_rev else float('nan'))


def count_reversals(points, cos_limit=-0.5):
    """Dem so lan vector van toc quay dau hon 120 do."""
    v = np.diff(points, axis=0)
    speed = np.linalg.norm(v, axis=1)
    ok = speed > 1e-5
    if ok.sum() < 3:
        return 0
    u = v[ok] / speed[ok][:, None]
    return int(np.sum(np.sum(u[:-1] * u[1:], axis=1) < cos_limit))


def row(tag, label, scope, m):
    if m is None:
        return f'{tag:16s}{label:7s}{scope:18s}  (khong du mau lien tuc)'
    return (f'{tag:16s}{label:7s}{scope:18s}{m["nseg"]:5d}{m["dur"]:7.0f}'
            f'{m["f_med"]:8.2f}{m["zero_pct"]:7.1f}{m["sat_pct"]:7.1f}'
            f'{m["ref_acc"]:8.3f}{m["ee_acc"]:8.3f}{m["rev_per_s"]:8.3f}'
            f'{m["n_release"]:6d}{m["drift"]:9.1f}{m["drift_max"]:8.1f}'
            f'{m["rev_per_release"]:9.2f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', nargs='+', action='append', required=True,
                    metavar=('NHAN', 'STEM'),
                    help='Nhan nhom roi den cac stem log, vi du: --label 2.5N 20260922_0930')
    ap.add_argument('--lead-limit', type=float, default=0.04,
                    help='max_command_lead_m dang dung (m)')
    args = ap.parse_args()

    print(f'{"trial":16s}{"nhan":7s}{"pham vi":18s}{"seg":>5s}{"giay":>7s}'
          f'{"F>0":>8s}{"zero%":>7s}{"sat%":>7s}{"refAcc":>8s}{"eeAcc":>8s}'
          f'{"rev/s":>8s}{"#nha":>6s}{"trôi_mm":>9s}{"max":>8s}{"rev/nha":>9s}')
    print('-' * 140)

    groups = {}
    for entry in args.label:
        label, stems = entry[0], entry[1:]
        for stem in stems:
            path = resolve(stem)
            df = pd.read_csv(path)
            missing = {'controller_state', 'control_phase'} - set(df.columns)
            if missing:
                print(f'{os.path.basename(path)[:16]:16s}{label:7s}'
                      f'  (thieu cot {sorted(missing)}, co the la schema cu)')
                continue
            t = df.ros_timestamp_ns.to_numpy() * 1e-9
            dt = float(np.median(np.diff(t)))
            running = (df.controller_state.astype(str).str.upper() == 'RUNNING').to_numpy()
            follower = running & (df.control_phase.astype(str) == 'FOLLOWER_PREDICTION').to_numpy()
            tag = os.path.basename(path)[len(PREFIX):len(PREFIX) + 15]
            for scope, mask in (('RUNNING (tat ca)', running),
                                ('FOLLOWER_PRED', follower)):
                m = metrics(df, mask, dt, args.lead_limit)
                print(row(tag, label, scope, m))
                if scope == 'FOLLOWER_PRED' and m:
                    groups.setdefault(label, []).append(m)
            print()

    if len(groups) > 1:
        print('=' * 140)
        print('Tong hop tren FOLLOWER_PREDICTION (median giua cac luot cung nhan):')
        keys = ('f_med', 'ref_acc', 'ee_acc', 'rev_per_s', 'drift', 'drift_max',
                'rev_per_release')
        print(f'{"nhan":8s}{"luot":>6s}' + ''.join(f'{k:>16s}' for k in keys))
        for label, rows in groups.items():
            vals = [np.nanmedian([r[k] for r in rows]) for k in keys]
            print(f'{label:8s}{len(rows):6d}' + ''.join(f'{v:16.3f}' for v in vals))
        print()
        print('Luu y: cac con so tren chi mo ta cac luot da chay. Voi 3 luot moi')
        print('nhan, chung la chi dau chu chua phai ket luan thong ke.')


if __name__ == '__main__':
    main()
