"""Run the production controller against an in-memory plant, with no DDS/motion IO."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
import cocarry_admittance_control.admittance_controller as module


class Pub:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeNode:
    def __init__(self, name):
        self.params = {}

    def declare_parameter(self, name, value):
        self.params[name] = value

    def get_parameter(self, name):
        return NS(value=self.params[name])

    def create_publisher(self, *args):
        return Pub()

    def create_subscription(self, *args):
        pass

    def create_service(self, *args):
        pass

    def create_timer(self, *args):
        pass

    def create_client(self, *args):
        return NS(service_is_ready=lambda: False)

    def get_logger(self):
        return NS(info=lambda *a, **k: None, warn=lambda *a, **k: None,
                  error=lambda *a, **k: None)

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=int(module.time.monotonic()*1e9),
                                to_msg=lambda: PoseStamped().header.stamp))


@pytest.fixture
def controller(tmp_path, monkeypatch):
    # Only swap transport base class. All controller methods, including __init__,
    # source validation, force watchdog and command lead limits, are production.
    tree = ast.parse(Path(module.__file__).read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    env = dict(vars(module), Node=FakeNode)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), module.__file__, 'exec'), env)
    monkeypatch.setattr(module.ManualHybrid, '_persist', module.ManualHybrid._persist)
    original = FakeNode.get_parameter
    monkeypatch.setattr(FakeNode, 'get_parameter', lambda self, key:
                        NS(value=str(tmp_path/'targets.json')) if key == 'hybrid_target_file'
                        else original(self, key))
    clock = [100.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    c = env['AdmittanceController3D']()
    p = PoseStamped()
    p.header.frame_id = 'base_link'
    p.pose.position.x, p.pose.position.y, p.pose.position.z = 0., .4, .5
    p.pose.orientation.w = 1.
    c._on_current_pose(p)
    c._capture_pose = p.pose
    c._capture_ee = np.array([0., .4, .5])
    c._ee_speed = 0.
    c._mode = 'prediction'
    c._manual.enabled = True
    c._manual.save(1, [.1, .4, .5], False)
    c._manual.save(2, [-.1, .4, .5], False)
    c._state = 'RUNNING'
    c._streamer_ready = c._axia_connected = c._axia_calibrated = True
    c._hybrid_state = c._requested_hybrid_state = 'FOLLOWER'
    c._force_time = clock[0]
    return c, clock


def cmd(c, action, **kw):
    c._manual_command(NS(data=json.dumps(dict(action=action, **kw))))


def tick(c, clock, follow=True):
    clock[0] += 1/15
    if follow and c._target_pub.messages:
        pose = c._target_pub.messages[-1]
    else:
        pose = PoseStamped(pose=c._current_pose)
    c._on_current_pose(pose)
    c._on_force(NS(vector=NS(x=0., y=0., z=0.)))
    p = c._current_pose.position
    c._on_prediction(NS(x=p.x, y=p.y-.4, z=p.z-.5, buffer_size=20, model_name='gru',
                        header=NS(stamp=NS(sec=1, nanosec=0))))
    c._control_tick()


def test_complete_two_selected_legs_and_handover(controller):
    c, clock = controller
    cmd(c, 'leader')
    assert c._manual.role == 'FOLLOWER'
    for target in (2, 1):
        cmd(c, 'select', target=target)
        assert c._manual.role == 'FOLLOWER'
        cmd(c, 'leader')
        tick(c, clock)
        assert c._manual.active == target
        cmd(c, 'select', target=3-target)
        assert c._manual.selected == target
        cmd(c, 'leader')
        for _ in range(160):
            tick(c, clock)
            assert not c._fault
            ref = c._target_pub.messages[-1].pose.position
            actual = c._current_pose.position
            assert np.linalg.norm([ref.x-actual.x, ref.y-actual.y, ref.z-actual.z]) <= c._max_command_lead+1e-9
            if c._manual.role == 'FOLLOWER':
                break
        assert c._manual.reason == 'reached'
        assert c._manual.selected == target
        tick(c, clock)
        assert not c._follower_realign
    assert c._manual.leg == 2


def test_cancel_enters_force_follower_and_preserves_target(controller):
    c, clock = controller
    cmd(c, 'select', target=1)
    cmd(c, 'leader')
    tick(c, clock)
    cmd(c, 'follower')
    assert c._prediction is None
    assert c._manual.selected == 1 and c._manual.reason == 'skipped_by_user'
    cmd(c, 'follower')
    tick(c, clock)
    assert not c._fault and not c._follower_realign
    p = c._current_pose.position
    r = c._target_pub.messages[-1].pose.position
    assert [p.x, p.y, p.z] == [r.x, r.y, r.z]


def test_stale_force_freezes_mjm_then_faults(controller):
    c, clock = controller
    cmd(c, 'select', target=1)
    cmd(c, 'leader')
    tick(c, clock)
    elapsed = c._manual.elapsed
    clock[0] += .3
    c._current_pose_time = clock[0]
    c._control_tick()
    assert c._manual.elapsed == elapsed and c._force_stale_active
    clock[0] += .3
    c._control_tick()
    assert 'Force data timeout' in c._fault
    assert c._manual.role == 'FOLLOWER'


def test_short_pose_dropout_holds_then_recovers(controller):
    c, clock = controller
    clock[0] += .30
    c._force_time = clock[0]
    c._control_tick()
    assert c._state == 'RUNNING'
    assert c._pose_stale_active
    held = c._target_pub.messages[-1].pose.position
    actual = c._current_pose.position
    assert np.allclose([held.x, held.y, held.z], [actual.x, actual.y, actual.z])

    tick(c, clock)
    assert c._state == 'RUNNING'
    assert not c._pose_stale_active


def test_long_pose_dropout_faults(controller):
    c, clock = controller
    clock[0] += .51
    c._force_time = clock[0]
    c._control_tick()
    assert 'Current EE pose timeout' in c._fault


def test_adaptive_z_deadzone_is_off_for_pure_z_and_ramps_in_for_xminus(controller):
    c, clock = controller
    c._force[:] = [0.0, 0.0, 3.73]
    assert np.allclose(c._effective_force_input(clock[0], .1), [0.0, 0.0, 3.73])

    c._force[:] = [-1.5, -2.5, 3.73]
    for _ in range(3):
        clock[0] += .1
        effective = c._effective_force_input(clock[0], .1)
    assert np.allclose(effective, [-1.5, -2.5, 1.73])
    assert c._z_deadzone_weight == 1.0
    published = c._effective_force_pub.messages[-1].vector
    assert np.allclose([published.x, published.y, published.z], effective)


def test_unreachable_actual_times_out_without_success(controller):
    c, clock = controller
    cmd(c, 'select', target=1)
    cmd(c, 'leader')
    for _ in range(220):
        tick(c, clock, follow=False)
        if c._fault:
            break
    assert 'target_not_reached' in c._fault
    assert c._manual.reason != 'reached'


def test_save_reset_rejected_during_run_and_legacy_ignored(controller):
    c, clock = controller
    before = dict(c._manual.targets)
    cmd(c, 'reset')
    cmd(c, 'save', target=1)
    assert c._manual.targets == before
    c._on_hybrid_state(NS(data='LEADER'))
    assert c._hybrid_state == 'FOLLOWER'
    c._on_prediction(NS(x=0., y=0., z=0., buffer_size=20, model_name='mjm'))
    assert c._prediction is None


def test_predictions_cannot_pull_force_follower(controller):
    c, clock = controller
    cmd(c, 'select', target=1)
    cmd(c, 'leader')
    tick(c, clock)
    cmd(c, 'follower')
    c._on_prediction(NS(x=9., y=9., z=9., buffer_size=20, model_name='gru',
                        header=NS(stamp=NS(sec=0, nanosec=0))))
    assert c._manual.force_follower and not c._follower_realign
    for _ in range(30):
        clock[0] += 1/15
        c._current_pose_time = clock[0]
        c._force_time = clock[0]
        c._control_tick()  # Deliberately no more predictions: observer only.
    assert not c._fault
    p = c._target_pub.messages[-1].pose.position
    assert np.allclose([p.x, p.y, p.z], [0., .4, .5], atol=1e-4)


def test_moving_handoff_keeps_velocity_and_release_follows_force(controller):
    c, clock = controller
    c._publish_reference(np.array([0., .4, .5]), np.zeros(3), np.array([0., .4, .5]))
    for i in range(1, 4):
        clock[0] += 1/15
        point = np.array([i*.08/15, .4, .5])
        c._publish_reference(point, np.zeros(3), point)
    c._force_time = c._current_pose_time = clock[0]
    cmd(c, 'select', target=1)
    v = c._emitted_velocity.copy()
    cmd(c, 'leader')
    tick(c, clock)
    assert c._manual.role == 'LEADER', c._manual.reason
    assert np.allclose(c._manual.bridge.state(0)[1], v)
    assert c._emitted_velocity[0] > .04  # No old near-zero first step.
    cmd(c, 'follower')
    inherited = c._free_admittance.error_velocity.copy()
    assert inherited[0] > .04
    for _ in range(30):
        clock[0] += 1/15
        if c._target_pub.messages:
            c._on_current_pose(c._target_pub.messages[-1])
        c._on_force(NS(vector=NS(x=-2., y=0., z=0.)))
        c._control_tick()
        assert not c._fault
    assert c._emitted_velocity[0] < 0  # Human can reverse, no GRU spring.


def test_auto_test_runs_once_and_pauses_during_force_hold(controller):
    c, clock = controller
    c._manual.test_mode = True
    c._manual.selected = 1
    for _ in range(30):
        tick(c, clock)
    elapsed = c._manual.test_elapsed
    clock[0] += .3
    c._current_pose_time = clock[0]
    c._control_tick()
    assert c._manual.test_elapsed == elapsed
    for _ in range(200):
        tick(c, clock)
        assert not c._fault
    assert c._manual.test_fired and c._manual.leg == 1
    assert c._manual.role == 'FOLLOWER' and c._manual.force_follower
