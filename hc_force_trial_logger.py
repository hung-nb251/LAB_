#!/usr/bin/env python3
"""Read-only HC register and ROS stream recorder. Run from workspace root."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import queue
import re
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, Vector3Stamped
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from motoros2_interfaces.srv import ReadMRegister


def decode(address, value, success):
    if not success or not 310 <= address <= 325 or 316 <= address <= 319:
        return None, None
    return (value - 10000) * 0.1, 'N' if 320 <= address <= 322 else 'Nm'


class Recorder(Node):
    KNOWN_PHASE = re.compile(
        r'^(setup|baseline|baseline_end|pre_start|stopped|'
        r'xplus_[1-9][0-9]*|xminus_[1-9][0-9]*|'
        r'release_(plus|minus)_[1-9][0-9]*)$')

    def __init__(self, args, directory):
        super().__init__('hc_force_trial_logger')
        self.args = args
        self.file = (directory / 'events.jsonl').open('x', buffering=1)
        self.phase = 'setup'
        self.commands = queue.Queue()
        self.pending = None
        self.paused = False
        self.index = 0
        self.scan = 0
        self.next_scan = 0.0
        self.last_seen = {}
        self.counts = {}
        self.addresses = list(range(320, 326)) if args.group == 'wrench' else [*range(310, 316), *range(320, 326), *range(330, 336), *range(340, 346)]
        self.client = self.create_client(ReadMRegister, '/read_mregister')
        for topic, cls in [('/joint_states', JointState), ('/axia/raw_wrench', WrenchStamped), ('/axia/human_force', Vector3Stamped), ('/axia/connected', Bool), ('/axia/calibrated', Bool), ('/run_status', Bool), ('/cocarry/status', String), ('/tf', TFMessage)]:
            self.create_subscription(cls, topic, lambda msg, t=topic: self.receive(t, msg), qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf_static', lambda msg: self.receive('/tf_static', msg), QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.01, self.tick)
        self.create_timer(5.0, self.health)
        self.emit('start', arguments=vars(args), addresses=self.addresses)
        threading.Thread(target=self.input_loop, daemon=True).start()
        print(f'LOG: {directory}\nNhập tên pha rồi Enter (baseline, xplus_1, release_1...). q = đóng log; pause/resume = dừng/tiếp tục đọc M. Không có lệnh robot.', flush=True)

    def emit(self, kind, **data):
        self.file.write(json.dumps(dict(kind=kind, phase=self.phase, wall_ns=time.time_ns(), monotonic_ns=time.monotonic_ns(), ros_ns=self.get_clock().now().nanoseconds, **data), ensure_ascii=False) + '\n')

    def receive(self, topic, msg):
        self.last_seen[topic] = time.monotonic()
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.emit('topic', topic=topic, message=message_to_ordereddict(msg))

    def input_loop(self):
        for line in sys.stdin:
            self.commands.put(line.strip())

    def health(self):
        ages = {t: round(time.monotonic()-s, 3) for t, s in self.last_seen.items()}
        self.emit('health', ages_sec=ages, counts=self.counts.copy(), register_paused=self.paused)
        print(f'phase={self.phase} scan={self.scan} paused={self.paused} joint_age={ages.get("/joint_states", "MISSING")} axia_age={ages.get("/axia/raw_wrench", "MISSING")}', flush=True)

    def tick(self):
        while not self.commands.empty():
            cmd = self.commands.get_nowait()
            if cmd == 'q':
                raise KeyboardInterrupt
            if cmd == 'pause':
                self.paused = True
            elif cmd == 'resume':
                if self.pending is not None:
                    print('Request cũ chưa trả về; không gửi chồng request.', flush=True)
                    continue
                self.paused = False
            elif cmd:
                self.phase = cmd
                if not self.KNOWN_PHASE.fullmatch(cmd):
                    print(
                        f'WARNING: marker không theo tên chuẩn: {cmd!r}. '
                        'Marker vẫn được lưu; kiểm tra trước khi thao tác.',
                        flush=True)
                print(f'MARKER: {self.phase}', flush=True)
            self.emit('marker', command=cmd)
        now = time.monotonic()
        if self.pending is not None:
            future, address, sent_ns, sent_phase, warned = self.pending
            if future.done():
                try:
                    response = future.result()
                    scaled, unit = decode(address, response.value, response.success)
                    self.emit('register', scan_id=self.scan, address=address, sent_monotonic_ns=sent_ns, sent_phase=sent_phase, latency_sec=(time.monotonic_ns()-sent_ns)/1e9, late=warned, response=message_to_ordereddict(response), scaled=scaled, unit=unit)
                except Exception as exc:
                    self.emit('register_error', address=address, scan_id=self.scan, error=str(exc))
                    self.paused = True
                self.pending = None
                self.index += 1
                if self.index == len(self.addresses):
                    self.emit('scan_end', scan_id=self.scan)
                    self.index = 0
                    self.next_scan = now + self.args.scan_gap
            elif not warned and (time.monotonic_ns()-sent_ns)/1e9 > self.args.timeout:
                self.emit('register_timeout', address=address, scan_id=self.scan)
                self.pending = (future, address, sent_ns, sent_phase, True)
                self.paused = True
                print('M-register timeout: dừng gửi mới, vẫn ghi Axia/joints. Chờ response cũ rồi nhập resume; timeout không hủy được request trên controller.', flush=True)
            return
        if self.paused or now < self.next_scan or not self.client.service_is_ready():
            return
        if self.index == 0:
            self.scan += 1
            self.emit('scan_start', scan_id=self.scan)
        address = self.addresses[self.index]
        sent_ns = time.monotonic_ns()
        future = self.client.call_async(ReadMRegister.Request(address=address))
        self.pending = (future, address, sent_ns, self.phase, False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', required=True)
    parser.add_argument('--mode', choices=['static', 'ground_truth'], required=True)
    parser.add_argument('--pose', required=True)
    parser.add_argument('--group', choices=['all', 'wrench'], default='all')
    parser.add_argument('--tool-number', type=int, required=True, help='Operator-reported, not read from controller')
    parser.add_argument('--notes', default='')
    parser.add_argument('--timeout', type=float, default=3.0)
    parser.add_argument('--scan-gap', type=float, default=0.5)
    parser.add_argument('--output', default='cocarry_logs/hc_force_trials')
    args = parser.parse_args()
    if not 0 < args.timeout < 120 or not 0 <= args.scan_gap < 120:
        parser.error('timeout must be (0,120), scan-gap [0,120) seconds')
    directory = Path(args.output) / (datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    directory.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parent
    snapshots = {}
    for name in ['hc_force_trial_logger.py', 'axia_sensor_ui.py', 'src/motoman_hc10dtp_support/urdf/hc10dtp_b00_macro_custom.xacro', 'src/cocarry_admittance_control/config/cocarry_admittance_params.yaml']:
        p = root / name
        if p.exists():
            content = p.read_text()
            snapshots[name] = {'sha256': hashlib.sha256(content.encode()).hexdigest(), 'content': content}
    (directory / 'metadata.json').write_text(json.dumps({'arguments': vars(args), 'environment': {k: os.environ.get(k) for k in ['ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY']}, 'source_snapshots': snapshots, 'note': 'Source defaults are not proof of runtime UI settings. Axia raw may already include hardware tare. Registers are sequential, not simultaneous.'}, ensure_ascii=False, indent=2))
    rclpy.init(args=[])
    node = Recorder(args, directory)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.emit('stop', incomplete_scan=bool(node.index or node.pending), counts=node.counts)
        node.file.close()
        node.destroy_node()
        rclpy.shutdown()
        print(f'Saved {directory}', flush=True)


if __name__ == '__main__':
    main()
