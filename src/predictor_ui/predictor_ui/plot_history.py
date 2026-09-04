"""Pure helpers for time-aligned dashboard plot histories."""


def append_time_window_xyz(buffers, timestamps, timestamp, point, window_sec):
    """Append XYZ and discard samples older than the requested time window."""
    timestamp = float(timestamp)
    window_sec = float(window_sec)
    if window_sec <= 0.0:
        raise ValueError('window_sec must be positive')
    if len(point) != 3:
        raise ValueError('point must contain XYZ')

    timestamps.append(timestamp)
    for axis, value in zip(('x', 'y', 'z'), point):
        buffers[axis].append(float(value))

    cutoff = timestamp - window_sec
    while timestamps and timestamps[0] < cutoff:
        timestamps.popleft()
        for axis in ('x', 'y', 'z'):
            buffers[axis].popleft()
