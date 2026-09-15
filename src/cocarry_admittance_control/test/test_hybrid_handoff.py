from types import MethodType, SimpleNamespace

import numpy as np

from cocarry_admittance_control.admittance_controller import AdmittanceController3D


class _Logger:
    def __init__(self):
        self.info_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def warn(self, _message):
        pass


class _Admittance:
    def __init__(self):
        self.reset_count = 0

    def reset(self):
        self.reset_count += 1


def _controller_harness():
    controller = SimpleNamespace(
        _manual=SimpleNamespace(enabled=False),
        _hybrid_state='FOLLOWER',
        _requested_hybrid_state='FOLLOWER',
        _leader_pending=False,
        _pending_mjm=None,
        _role_change_time=0.0,
        _state='RUNNING',
        _mode='prediction',
        _prediction=np.array((0.1, 0.2, 0.3)),
        _prediction_time=1.0,
        _prediction_buffer_size=10,
        _prediction_source='svgp',
        _prediction_timeout=0.5,
        _follower_realign=False,
        _admittance=_Admittance(),
        get_logger=lambda: _Logger(),
        _set_fault=lambda _reason: None,
    )
    controller._activate_leader = MethodType(
        AdmittanceController3D._activate_leader, controller)
    return controller


def _hybrid_state(state):
    return SimpleNamespace(data=state)


def _prediction(source, xyz):
    return SimpleNamespace(
        x=xyz[0], y=xyz[1], z=xyz[2], buffer_size=10, model_name=source)


def test_leader_request_keeps_follower_until_mjm_arrives():
    controller = _controller_harness()
    old_prediction = controller._prediction.copy()

    AdmittanceController3D._on_hybrid_state(
        controller, _hybrid_state('LEADER'))

    assert controller._hybrid_state == 'FOLLOWER'
    assert controller._leader_pending
    assert np.allclose(controller._prediction, old_prediction)
    assert controller._prediction_source == 'svgp'
    assert controller._admittance.reset_count == 0

    AdmittanceController3D._on_prediction(
        controller, _prediction('mjm', (0.12, 0.22, 0.32)))

    assert controller._hybrid_state == 'LEADER'
    assert not controller._leader_pending
    assert controller._prediction_source == 'mjm'
    assert np.allclose(controller._prediction, (0.12, 0.22, 0.32))
    assert controller._admittance.reset_count == 1


def test_early_mjm_is_held_separately_until_leader_request():
    controller = _controller_harness()
    old_prediction = controller._prediction.copy()

    AdmittanceController3D._on_prediction(
        controller, _prediction('mjm', (0.15, 0.25, 0.35)))

    assert controller._hybrid_state == 'FOLLOWER'
    assert controller._prediction_source == 'svgp'
    assert np.allclose(controller._prediction, old_prediction)
    assert controller._pending_mjm is not None

    AdmittanceController3D._on_hybrid_state(
        controller, _hybrid_state('LEADER'))

    assert controller._hybrid_state == 'LEADER'
    assert controller._prediction_source == 'mjm'
    assert np.allclose(controller._prediction, (0.15, 0.25, 0.35))
    assert controller._pending_mjm is None
    assert controller._admittance.reset_count == 1
