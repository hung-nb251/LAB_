"""Atomic force-estimate records for diagnostics, never robot control."""
import math


def validate_force_sample(sample):
    if not isinstance(sample, dict) or sample.get('schema') != 1:
        raise ValueError('Unsupported force sample schema')
    if not isinstance(sample.get('stamp_ns'), int):
        raise ValueError('Missing force source timestamp')
    for key, size in [('force', 3), ('force_unfiltered', 3), ('torque_nm', 6),
                      ('position', 6), ('effort_raw', 6), ('diagnostics', 4),
                      ('scale_nm_per_effort', 6), ('bias_nm', 6)]:
        value = sample.get(key)
        if value is not None and (len(value) != size or not all(
                isinstance(v, (int, float)) and math.isfinite(v) for v in value)):
            raise ValueError(f'Invalid force sample field {key}')
    if sample.get('frame') != 'base_link' or not isinstance(sample.get('status'), str):
        raise ValueError('Invalid force frame/status')
    return sample


def force_log_record(sample, now_ns, receipt_age_sec, timeout_sec):
    """Never repeat an old valid force after invalid input or publisher loss."""
    if sample is None:
        return {'status': 'NO_SAMPLE', 'role_valid': False, 'age_ms': ''}
    record = dict(sample)
    age_ms = (now_ns - sample['stamp_ns']) / 1e6
    record['age_ms'] = age_ms
    stale = (sample['stamp_ns'] <= 0 or age_ms < 0 or
             age_ms > timeout_sec * 1000 or receipt_age_sec > timeout_sec)
    invalid = sample['status'].startswith(('INVALID:', 'RAW_ONLY:'))
    if stale:
        record['status'] = 'STALE:' + sample['status']
        for key in ['force_unfiltered', 'torque_nm', 'position', 'effort_raw', 'diagnostics']:
            record[key] = None
    if stale or invalid:
        record['force'] = None
        record['role_valid'] = False
    return record
