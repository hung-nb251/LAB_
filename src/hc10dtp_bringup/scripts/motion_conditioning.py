"""Pure queue conditioning helpers; no ROS calls or hardware access."""
import math


def synchronized_joint_step(previous, target, dt, limits):
    """Advance all joints by the same fraction, bounded by each velocity cap.

    Preserves direction in joint space, not an exact Cartesian straight line.
    This is a velocity limiter, not a joint acceleration/jerk guarantee.
    """
    if (len(previous) != 6 or len(target) != 6 or len(limits) != 6
            or not math.isfinite(dt) or dt <= 0
            or not all(math.isfinite(x) for x in (*previous, *target, *limits))
            or any(x <= 0 for x in limits)):
        raise ValueError('Expected finite six-axis data and positive dt/limits')
    delta = [b - a for a, b in zip(previous, target)]
    scale = min([1.0] + [limit * dt / abs(d) for d, limit in zip(delta, limits) if d])
    velocity = [d * scale / dt for d in delta]
    position = [a + d * scale for a, d in zip(previous, delta)]
    return position, velocity, scale


def sample_timed_position(history, anchor, now_ns):
    """Consume due XYZ points and interpolate towards the next accepted point.

    Returns (sample, anchor). Does not extrapolate, alter the queue epoch or
    compensate measured lag. Linear interpolation is an approximation between
    FK endpoints, not a reconstruction of controller spline interpolation.
    Caller holds the history lock; anchor is the last *endpoint*, never a sample.
    """
    while history and history[0][0] <= now_ns:
        anchor = history.popleft()
    if anchor is None:
        return None, None
    due, position = anchor
    if not history or now_ns <= due:
        return list(position), anchor
    future_due, future = history[0]
    if future_due <= due:
        return list(position), anchor
    fraction = min(1.0, (now_ns - due) / (future_due - due))
    return [a + fraction * (b - a) for a, b in zip(position, future)], anchor
