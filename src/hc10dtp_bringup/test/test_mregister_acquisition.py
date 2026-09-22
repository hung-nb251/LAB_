"""Acquisition-loop tests for mregister_force_node.

Only the register reading loop is exercised. The calibration, baseline and
wrench recovery paths are left to test_sensorless_force_math.py. No ROS graph
is created, so these never touch the live domain.
"""
import ast
import math
import time
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

METHODS = ('_tick', '_send_request', '_restart_scan', '_publish_stall',
           '_acquisition_block', '_on_set_parameters', '_sample_template')
ADDRESSES = tuple(range(310, 316))


def methods():
    path = Path(__file__).parents[1] / 'scripts/mregister_force_node.py'
    tree = ast.parse(path.read_text())
    wanted = [n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name in METHODS]
    assert len(wanted) == len(METHODS), 'method set changed'
    env = {'time': time, 'math': math, 'np': np, 'ADDRESSES': ADDRESSES,
           'ReadMRegister': NS(Request=lambda address: NS(address=address)),
           'SetParametersResult': lambda successful, reason=None: NS(
               successful=successful, reason=reason)}
    exec(compile(ast.Module(body=wanted, type_ignores=[]), str(path), 'exec'), env)
    return env


class Future:
    def __init__(self):
        self.response = None
        self.exception = None

    def done(self):
        return self.response is not None or self.exception is not None

    def result(self):
        if self.exception is not None:
            raise self.exception
        return self.response


class Client:
    def __init__(self, ready=True):
        self.ready = ready
        self.sent = []
        self.futures = []
        self.removed = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.sent.append(request.address)
        future = Future()
        self.futures.append(future)
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


def node(**overrides):
    env = methods()
    client = Client()
    completed = []
    published = []
    logged = []
    s = NS(_client=client, _address_index=0, _scan_values=[0.0] * 6,
           _scan_raw=[0] * 6, _scan_first_ns=0, _pending=None, _next_scan=0.0,
           _last_status='', _scan_id=0, _scan_registers=[], _scan_restarts=0,
           _timeout_active=False,
           _errors={'service_error': 0, 'not_success': 0,
                    'request_timeout': 0, 'abandoned_request': 0},
           _scan_gap=0.05, _request_timeout=1.0, _request_recovery=2.0,
           _latest_q=[0.0] * 6, _latest_joint_monotonic=time.monotonic(),
           _baseline=None, _q0=None, _scale=np.ones(12), _model_path='model.json',
           _clock_ns=1_000_000_000)
    s.__dict__.update(overrides)
    s.get_clock = lambda: NS(now=lambda: NS(nanoseconds=s._clock_ns))
    s.get_logger = lambda: NS(error=logged.append, info=logged.append,
                              warning=logged.append)
    s._complete_scan = lambda stamp, span, end: completed.append(
        {'stamp': stamp, 'span_ms': span, 'end': end,
         'values': list(s._scan_values), 'raw': list(s._scan_raw),
         'acquisition': s._acquisition_block(span, end)})
    s._publish_sample = published.append
    for name in METHODS:
        setattr(s, name, env[name].__get__(s, NS))
    return s, client, completed, published, logged


def answer(client, value, clock_step=20_000_000, node=None):
    client.futures[-1].response = NS(success=True, value=value)
    if node is not None:
        node._clock_ns += clock_step


def test_full_scan_emits_one_vector_with_per_register_provenance():
    s, client, completed, _, _ = node()
    s._tick()
    assert client.sent == [310]
    for i, raw in enumerate((10090, 9956, 10030, 9968, 9978, 10006)):
        answer(client, raw, node=s)
        s._tick()
    assert len(completed) == 1
    scan = completed[0]
    assert scan['raw'] == [10090, 9956, 10030, 9968, 9978, 10006]
    assert scan['values'] == pytest.approx([9.0, -4.4, 3.0, -3.2, -2.2, 0.6])
    regs = scan['acquisition']['registers']
    assert [r['address'] for r in regs] == list(ADDRESSES)
    assert all(r['rtt_ms'] > 0 and r['request_ns'] < r['response_ns'] for r in regs)
    assert scan['acquisition']['scan_id'] == 1
    assert scan['span_ms'] == pytest.approx(120.0)


def test_next_register_is_sent_in_the_same_tick_as_the_response():
    s, client, _, _, _ = node()
    s._tick()
    answer(client, 10000, node=s)
    s._tick()
    assert client.sent == [310, 311], 'a whole timer period was wasted per register'


def test_timeout_publishes_an_invalid_sample_instead_of_going_silent():
    s, client, _, published, _ = node()
    s._tick()
    s._pending = (client.futures[-1], time.monotonic() - 1.5,
                  s._clock_ns)
    s._tick()
    assert [p['status'] for p in published] == ['INVALID:mregister_request_timeout']
    assert published[0]['acquisition']['errors_cumulative']['request_timeout'] == 1
    assert client.removed == [], 'must not abandon before the recovery window'


def test_recovery_drops_the_request_then_restarts_the_scan():
    s, client, _, _, _ = node()
    s._tick()
    stale = client.futures[-1]
    s._pending = (stale, time.monotonic() - 4.0, s._clock_ns)
    s._tick()
    assert client.removed == [stale], 'unanswered request must be dropped'
    assert s._pending is None and s._address_index == 0
    assert s._errors['abandoned_request'] == 1
    assert s._scan_registers == []
    s._next_scan = 0.0
    s._tick()
    assert client.sent == [310, 310]
    assert len(client.futures) == 2, 'exactly one request outstanding after recovery'


def test_recovered_reader_resumes_when_the_controller_answers_again():
    s, client, completed, _, _ = node()
    s._tick()
    s._pending = (client.futures[-1], time.monotonic() - 4.0, s._clock_ns)
    s._tick()
    s._next_scan = 0.0
    for _ in range(6):
        s._tick()
        answer(client, 10000, node=s)
        s._tick()
    assert len(completed) == 1


def test_failed_response_restarts_the_scan_without_emitting_a_vector():
    s, client, completed, published, _ = node()
    s._tick()
    answer(client, 10000, node=s)
    s._tick()
    client.futures[-1].response = NS(success=False, value=0)
    s._tick()
    assert completed == [] and published == []
    assert s._address_index == 0 and s._errors['not_success'] == 1


def test_service_exception_restarts_the_scan():
    s, client, completed, _, logged = node()
    s._tick()
    client.futures[-1].exception = RuntimeError('service gone')
    s._tick()
    assert completed == [] and s._address_index == 0
    assert s._errors['service_error'] == 1
    assert any('service gone' in str(m) for m in logged)


def test_scan_gap_is_runtime_tunable_and_bad_values_are_rejected():
    s, _, _, _, _ = node()
    assert s._on_set_parameters([NS(name='scan_gap_sec', value=0.0)]).successful
    assert s._scan_gap == 0.0
    bad = s._on_set_parameters([NS(name='request_timeout_sec', value=0.0)])
    assert not bad.successful
    assert not s._on_set_parameters([NS(name='deadband_n', value=1.0)]).successful
    assert not s._on_set_parameters(
        [NS(name='scan_gap_sec', value=float('nan'))]).successful
    assert s._scan_gap == 0.0, 'a rejected set must not be applied'


def test_nothing_is_sent_while_the_service_is_down():
    s, client, _, _, _ = node()
    client.ready = False
    s._tick()
    assert client.sent == []
