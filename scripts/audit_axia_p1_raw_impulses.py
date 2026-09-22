#!/usr/bin/env python3
"""Scan complete source raw logs for impulse-like events, without force relabeling."""
import argparse
import json
from pathlib import Path

import numpy as np

from benchmark_axia_p1 import ROOT, impulse_like_indices, save, p0, stats


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--prepared',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args(); args.output.mkdir(parents=True,exist_ok=False)
    rows=[]
    for source in json.loads((args.prepared/'source_inventory.json').read_text()):
        path=ROOT/source['path']
        if p0.digest(path)!=source['sha256']: raise ValueError('Source changed')
        samples=[]; first=None
        for line in path.open():
            e=json.loads(line)
            if first is None: first=e['monotonic_ns']
            if e['kind']=='topic' and e.get('topic')=='/axia/raw_wrench':
                samples.append([(e['monotonic_ns']-first)/1e9,
                                *[e['message']['wrench']['force'][a] for a in 'xyz']])
        a=np.asarray(samples); t=a[:,0]; raw=a[:,1:]
        finite=np.isfinite(raw).all(axis=1)
        # Do not label time-gap boundaries as spikes. Only annotations, not safety signals.
        valid=finite.copy(); valid[:5]=False; valid[-5:]=False
        for i in np.flatnonzero((np.diff(t)>.05)|(np.diff(t)<=0)):
            valid[max(0,i-5):min(len(t),i+6)]=False
        clean=raw.copy(); clean[~finite]=0.
        ix,_,amplitude=impulse_like_indices(clean,valid)
        event=[dict(time_s=float(t[i]),amplitude_n=float(amplitude[i]),raw_force_n=raw[i].tolist()) for i in ix]
        rows.append(dict(**source,full_raw_samples=len(a),impulse_like_count=len(ix),
                         amplitude_n=stats(amplitude[ix]),events=event))
        print(source['trial'],len(a),'raw samples;',len(ix),'impulse-like events',flush=True)
    save(args.output/'raw_impulse_audit.json',dict(
        source_files=len(rows),raw_samples=sum(r['full_raw_samples'] for r in rows),
        impulse_like_events=sum(r['impulse_like_count'] for r in rows),
        criterion='Single sample >max(0.5 N, 6 robust sigma) from centered median9; neighbors return within .15 N.',
        limitations=['Offline annotation uses future neighbors; never used by candidate filter.',
                     'A short physical force can satisfy the criterion; not proven electrical noise.',
                     'No event under this criterion does not exclude bursts, smaller spikes, or TF/compensation artifacts.',
                     'Full raw files include setup and uncalibrated periods; these events are not added to rest metrics.'],
        files=rows))


if __name__=='__main__': main()
