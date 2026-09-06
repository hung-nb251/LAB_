import numpy as np
import pytest

from trajectory_predictor.svgp_numpy import SVGPNumpyRunner


def _write_model(path, *, kernel_type='Matern52', lengthscales=(1.0, 2.0)):
    np.savez(
        path,
        Z=np.array([[0.0, 0.0], [1.0, 1.0]]),
        alpha=np.array([[1.0, 2.0, 3.0], [-1.0, 0.5, 0.25]]),
        kernel_variance=np.array([2.0]),
        kernel_lengthscales=np.asarray(lengthscales),
        kernel_type=np.array([kernel_type]),
    )


def test_matern52_prediction_has_expected_shape_and_value_at_inducing_point(tmp_path):
    path = tmp_path / 'svgp_model.npz'
    _write_model(path)
    runner = SVGPNumpyRunner(path, input_dim=2, output_dim=3)

    mean, variance = runner.predict_f([[0.0, 0.0]])

    squared_distance = 1.0 + 0.25
    scaled_radius = np.sqrt(5.0 * squared_distance)
    cross_kernel = 2.0 * (
        1.0 + scaled_radius + (5.0 / 3.0) * squared_distance
    ) * np.exp(-scaled_radius)
    expected = 2.0 * np.array([1.0, 2.0, 3.0])
    expected += cross_kernel * np.array([-1.0, 0.5, 0.25])
    assert variance is None
    assert mean.shape == (1, 3)
    assert np.allclose(mean[0], expected)


def test_model_dimension_mismatch_is_rejected(tmp_path):
    path = tmp_path / 'svgp_model.npz'
    _write_model(path)

    with pytest.raises(ValueError, match='input dimension'):
        SVGPNumpyRunner(path, input_dim=30, output_dim=3)


def test_unknown_kernel_is_rejected(tmp_path):
    path = tmp_path / 'svgp_model.npz'
    _write_model(path, kernel_type='UnknownKernel')

    with pytest.raises(ValueError, match='Unsupported SVGP kernel'):
        SVGPNumpyRunner(path, input_dim=2, output_dim=3)
