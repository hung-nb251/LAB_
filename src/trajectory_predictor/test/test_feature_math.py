import numpy as np
import pytest

from trajectory_predictor.feature_math import per_sample_displacement


def test_first_sample_has_zero_displacement_feature():
    assert np.allclose(per_sample_displacement((0.1, -0.2, 0.3)), 0.0)


def test_delta_position_matches_training_definition():
    previous = (0.10, -0.20, 0.30)
    current = (0.11, -0.23, 0.305)
    assert np.allclose(
        per_sample_displacement(current, previous),
        (0.01, -0.03, 0.005),
    )


def test_invalid_position_is_rejected():
    with pytest.raises(ValueError):
        per_sample_displacement((np.nan, 0.0, 0.0))
