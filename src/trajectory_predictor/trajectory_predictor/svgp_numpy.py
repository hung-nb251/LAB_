"""Fast NumPy-only inference for exported GPflow SVGP mean predictions."""

from pathlib import Path

import numpy as np


class SVGPNumpyRunner:
    """Evaluate an exported SVGP posterior mean without TensorFlow/GPflow ops."""

    _SUPPORTED_KERNELS = {'Matern52', 'SquaredExponential'}

    def __init__(self, model_path, input_dim=None, output_dim=3):
        path = Path(model_path)
        with np.load(path, allow_pickle=False) as data:
            self._z = np.asarray(data['Z'], dtype=np.float64)
            self._alpha = np.asarray(data['alpha'], dtype=np.float64)
            variance = np.asarray(data['kernel_variance'], dtype=np.float64)
            self._lengthscales = np.asarray(
                data['kernel_lengthscales'], dtype=np.float64)
            kernel_type = np.asarray(data['kernel_type'])

        if self._z.ndim != 2 or self._alpha.ndim != 2:
            raise ValueError('SVGP NPZ requires 2-D Z and alpha arrays')
        if self._z.shape[0] != self._alpha.shape[0]:
            raise ValueError('SVGP NPZ Z/alpha inducing-point counts differ')
        if self._lengthscales.shape != (self._z.shape[1],):
            raise ValueError('SVGP NPZ lengthscale dimension does not match Z')
        if input_dim is not None and self._z.shape[1] != int(input_dim):
            raise ValueError(
                f'SVGP NPZ input dimension {self._z.shape[1]} does not match '
                f'runtime {int(input_dim)}')
        if output_dim is not None and self._alpha.shape[1] != int(output_dim):
            raise ValueError(
                f'SVGP NPZ output dimension {self._alpha.shape[1]} does not '
                f'match runtime {int(output_dim)}')
        if variance.size != 1:
            raise ValueError('SVGP NPZ kernel_variance must contain one value')

        self._variance = float(variance.reshape(-1)[0])
        self._kernel_type = str(kernel_type.reshape(-1)[0])
        if self._kernel_type not in self._SUPPORTED_KERNELS:
            raise ValueError(f'Unsupported SVGP kernel: {self._kernel_type}')
        if self._variance <= 0.0 or np.any(self._lengthscales <= 0.0):
            raise ValueError('SVGP kernel variance and lengthscales must be positive')
        arrays = (self._z, self._alpha, self._lengthscales)
        if not all(np.all(np.isfinite(values)) for values in arrays):
            raise ValueError('SVGP NPZ contains non-finite values')

    def predict_f(self, inputs):
        """Return posterior mean and ``None`` variance, matching worker usage."""
        values = np.asarray(inputs, dtype=np.float64)
        if values.ndim != 2 or values.shape[1] != self._z.shape[1]:
            raise ValueError(
                f'SVGP input shape {values.shape} must be (batch, '
                f'{self._z.shape[1]})')
        if not np.all(np.isfinite(values)):
            raise ValueError('SVGP input contains non-finite values')
        return self._kernel_matrix(values, self._z) @ self._alpha, None

    def _kernel_matrix(self, inputs, inducing):
        scaled_inputs = inputs / self._lengthscales
        scaled_inducing = inducing / self._lengthscales
        squared_distance = (
            np.sum(scaled_inputs ** 2, axis=1, keepdims=True)
            - 2.0 * scaled_inputs @ scaled_inducing.T
            + np.sum(scaled_inducing ** 2, axis=1)
        )
        squared_distance = np.maximum(squared_distance, 0.0)
        if self._kernel_type == 'SquaredExponential':
            return self._variance * np.exp(-0.5 * squared_distance)

        radius = np.sqrt(squared_distance)
        scaled_radius = np.sqrt(5.0) * radius
        return self._variance * (
            1.0 + scaled_radius + (5.0 / 3.0) * squared_distance
        ) * np.exp(-scaled_radius)
