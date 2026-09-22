#!/usr/bin/env python3
"""Do ripple cua co-carry bang so DOT DAO DONG tren thoi gian buong tay.

Vi sao lai la chi so nay, chu khong phai dem so lan doi chieu:

- Mot dot dao dong nha ra nhieu lan doi chieu lien tiep o cung mot cho, nen dem
  lan doi chieu se khuech dai mot su kien duy nhat thanh mot dinh gia tao. Ban
  do khong gian dung trong ngay 22/09 chinh la nan nhan cua bay nay.
- Dao dong gan nhu chi xay ra khi nguoi buong tay: tro khang canh tay la bo
  giam chan cua vong GRU -> nominal -> admittance -> robot. Vi vay mau so dung
  la THOI GIAN BUONG TAY, khong phai do dai luot.

Muc nen do duoc tren 45 luot GRU ngay 21-22/09: 1 dot moi ~46 giay buong tay.
Ground Truth: 176 giay buong tay, 0 dot. Mot thay doi duoc coi la co tac dung
khi no cho 0 dot tren >=100 giay buong tay tich luy.

Cach dung:
    python3 scripts/report_ripple_episodes.py                 # tat ca log hom nay
    python3 scripts/report_ripple_episodes.py 20260922_1004   # loc theo tien to
"""
import glob
import os
import sys

import numpy as np
import pandas as pd

LOG_DIR = os.path.expanduser('~/cocarry_ws/cocarry_logs')
PREFIX = 'cocarry_admittance_3d_'
MIN_HOLD_SEC = 1.5      # doan buong tay ngan hon thi khong tinh phoi nhiem
GAP_SEC = 2.0           # cac lan doi chieu cach nhau xa hon thi thuoc hai dot
MIN_REV_PER_EPISODE = 3  # duoi nguong nay chi la mot lan doi huong binh thuong


def runs_of(mask):
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return []
    return np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)


def analyse(path):
    d = pd.read_csv(path)
    if not {'actual_ee_x', 'f_human_x', 'nominal_xd'} <= set(d.columns):
        return None
    r = (d[d.controller_state.astype(str).str.upper() == 'RUNNING'].reset_index(drop=True)
         if 'controller_state' in d.columns else d.reset_index(drop=True))
    if len(r) < 300:
        return None
    t = r.ros_timestamp_ns.to_numpy() * 1e-9
    t = t - t[0]
    dt = float(np.median(np.diff(t)))
    ee = r[['actual_ee_x', 'actual_ee_y', 'actual_ee_z']].to_numpy()
    nom = r[['nominal_xd', 'nominal_yd', 'nominal_zd']].to_numpy()
    mag = np.linalg.norm(r[['f_human_x', 'f_human_y', 'f_human_z']].to_numpy(), axis=1)

    mode = 'GRU' if np.linalg.norm(nom - nom[0], axis=1).max() > 0.005 else 'GT'
    exposure = sum(len(s) * dt for s in runs_of(mag == 0.0) if len(s) * dt >= MIN_HOLD_SEC)

    v = np.diff(ee, axis=0)
    sp = np.linalg.norm(v, axis=1)
    ok = sp > 1e-5
    episodes = []
    if ok.sum() > 3:
        idx = np.flatnonzero(ok)
        u = v[ok] / sp[ok][:, None]
        at = idx[1:][np.sum(u[:-1] * u[1:], axis=1) < -0.5]
        if at.size:
            for g in np.split(at, np.flatnonzero(np.diff(t[at]) > GAP_SEC) + 1):
                if len(g) >= MIN_REV_PER_EPISODE:
                    episodes.append((t[g[0]], t[g[-1]], len(g)))

    age = (float(np.median(r.prediction_age_ms))
           if 'prediction_age_ms' in r.columns else float('nan'))

    # Chi so thu hai, doc lap voi cac dot: do rung LIEN TUC cua reference gui
    # xuong robot. tau=0.2 ngay 22/09 lam no gap doi (0.76 so voi 0.36-0.40) ma
    # khong sinh them dot nao, nen dem dot thoi se bo sot che do hong do.
    xr = r[['reference_xr', 'reference_yr', 'reference_zr']].to_numpy()
    ref_acc = float(np.linalg.norm(np.diff(xr, 2, axis=0) / dt ** 2, axis=1).std())
    return dict(mode=mode, dur=t[-1], exposure=exposure, episodes=episodes,
                pred_age=age, ref_acc=ref_acc,
                hold_pct=100.0 * float(np.mean(mag == 0.0)))


def main():
    pat = sys.argv[1] if len(sys.argv) > 1 else ''
    files = sorted(glob.glob(os.path.join(LOG_DIR, f'{PREFIX}*{pat}*.csv')))
    if not files:
        print(f'Khong tim thay log khop "{pat}" trong {LOG_DIR}')
        return

    print(f'{"luot":>16s}{"mode":>5s}{"dai(s)":>8s}{"buong tay(s)":>14s}'
          f'{"predAge":>9s}{"refAcc":>8s}{"dot":>5s}   chi tiet cac dot')
    print('-' * 108)
    tot = {'GRU': [0.0, 0], 'GT': [0.0, 0]}
    for f in files:
        res = analyse(f)
        if res is None:
            continue
        stem = os.path.basename(f)[len(PREFIX):-4]
        detail = '  '.join(f'{a:.0f}-{b:.0f}s({n})' for a, b, n in res['episodes'])
        print(f'{stem:>16s}{res["mode"]:>5s}{res["dur"]:8.0f}{res["exposure"]:14.1f}'
              f'{res["pred_age"]:9.1f}{res["ref_acc"]:8.3f}{len(res["episodes"]):5d}'
              f'   {detail}')
        tot[res['mode']][0] += res['exposure']
        tot[res['mode']][1] += len(res['episodes'])

    print()
    for mode, (expo, ep) in tot.items():
        if expo <= 0:
            continue
        rate = f'1 dot moi {expo / ep:.0f} s' if ep else 'khong co dot nao'
        print(f'{mode}: {expo:.0f} s buong tay, {ep} dot  ->  {rate}')
    expo, ep = tot['GRU']
    if expo > 0:
        print(f'\nMuc nen lich su (21-22/09): 1 dot moi 46 s buong tay.')
        print(f'Ky vong o muc nen cho {expo:.0f} s nay: {expo/46:.1f} dot. '
              f'Thuc te: {ep}.')
        if expo < 100:
            print(f'Chua du phoi nhiem de ket luan: can >=100 s, moi co {expo:.0f} s.')
    print('\nrefAcc (rung lien tuc cua reference): 0.36-0.40 la muc binh thuong '
          'o tau 0.4-0.8;\n0.76 do duoc o tau 0.2 ngay 22/09 va nguoi van hanh '
          'thay ripple ro ret.')


if __name__ == '__main__':
    main()
