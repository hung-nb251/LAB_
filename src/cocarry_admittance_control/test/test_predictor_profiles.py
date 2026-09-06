import pytest

from cocarry_admittance_control.predictor_profiles import predictor_profile


MODEL_DIRS = {
    'svgp': '/models/svgp',
    'gru': '/models/gru',
}


def test_svgp_profile_keeps_existing_robot_ee_contract():
    profile = predictor_profile('svgp', MODEL_DIRS)
    assert profile['model_dir'] == '/models/svgp'
    assert profile['default_model'] == 'svgp'
    assert profile['model_files.svgp'] == 'svgp_model.npz'
    assert profile['window_size'] == 10
    assert profile['num_features'] == 3
    assert profile['filter.enabled'] is True


def test_gru_profile_matches_training_metadata():
    profile = predictor_profile('GRU', MODEL_DIRS)
    assert profile['model_dir'] == '/models/gru'
    assert profile['default_model'] == 'gru'
    assert profile['model_files.gru'] == 'gru_joint_Ts5_float16.tflite'
    assert profile['window_size'] == 20
    assert profile['num_features'] == 6
    assert profile['filter.enabled'] is False
    assert profile['velocity_feature_mode'] == 'delta_position'


def test_explicit_model_directory_overrides_selected_profile_only():
    profile = predictor_profile('gru', MODEL_DIRS, '/tmp/custom-gru')
    assert profile['model_dir'] == '/tmp/custom-gru'
    assert profile['default_model'] == 'gru'


def test_unknown_backend_is_rejected():
    with pytest.raises(ValueError, match='Unsupported prediction_model'):
        predictor_profile('lstm', MODEL_DIRS)
