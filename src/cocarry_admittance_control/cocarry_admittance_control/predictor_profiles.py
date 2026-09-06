"""Predictor model profiles for the robot-EE co-carry pipeline."""


PREDICTOR_PROFILES = {
    'svgp': {
        'default_model': 'svgp',
        'model_files.svgp': 'svgp_model.npz',
        'window_size': 10,
        'num_features': 3,
        # The co-carry base configuration defaults to raw GRU output.  Restore
        # the validated SVGP smoothing contract whenever SVGP is selected.
        'filter.enabled': True,
        # Unused by a three-feature model.  Keep the historical default
        # explicit so switching profiles cannot affect the camera pipeline.
        'velocity_feature_mode': 'legacy_16hz_ema',
    },
    'gru': {
        'default_model': 'gru',
        'model_files.gru': 'gru_joint_Ts5_float16.tflite',
        'window_size': 20,
        'num_features': 6,
        # The robot-EE GRU was trained against raw targets.  Its output is
        # already smooth enough and EMA adds measurable prediction lag.
        'filter.enabled': False,
        # Training uses delta_p[k] = p[k] - p[k-1] after 15 Hz resampling.
        'velocity_feature_mode': 'delta_position',
    },
}


def predictor_profile(name, model_directories, model_dir_override=''):
    """Return a fresh ROS-parameter dictionary for one predictor backend."""
    backend = str(name).strip().lower()
    if backend not in PREDICTOR_PROFILES:
        choices = ', '.join(sorted(PREDICTOR_PROFILES))
        raise ValueError(
            f"Unsupported prediction_model '{name}'; expected one of: {choices}")
    if backend not in model_directories:
        raise ValueError(f"Missing model directory for backend '{backend}'")

    selected_dir = str(model_dir_override).strip()
    if not selected_dir:
        selected_dir = str(model_directories[backend]).strip()
    if not selected_dir:
        raise ValueError(f"Empty model directory for backend '{backend}'")

    profile = dict(PREDICTOR_PROFILES[backend])
    profile['model_dir'] = selected_dir
    profile['scaler_x_file'] = 'scaler_x.pkl'
    profile['scaler_y_file'] = 'scaler_y.pkl'
    return profile
