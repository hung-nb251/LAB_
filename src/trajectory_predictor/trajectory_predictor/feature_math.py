"""Pure feature calculations shared by predictor runtime and tests."""

import numpy as np


def per_sample_displacement(current, previous=None):
    """Return delta_p for one uniformly sampled XYZ observation.

    The first observation has no predecessor and therefore receives the same
    zero velocity feature used by the robot-EE training pipeline.
    """
    current_array = np.asarray(current, dtype=np.float64)
    if current_array.shape != (3,) or not np.all(np.isfinite(current_array)):
        raise ValueError('current must contain three finite XYZ values')
    if previous is None:
        return np.zeros(3, dtype=np.float64)
    previous_array = np.asarray(previous, dtype=np.float64)
    if previous_array.shape != (3,) or not np.all(np.isfinite(previous_array)):
        raise ValueError('previous must contain three finite XYZ values')
    return current_array - previous_array
