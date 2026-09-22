#!/usr/bin/env python3
"""Interactive marker publisher for hc_force_trial_logger.py."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/hc_force_trial/marker')
    args = parser.parse_args()

    rclpy.init(args=[])
    node = Node('hc_force_marker_cli')
    publisher = node.create_publisher(String, args.topic, 10)
    deadline = time.monotonic() + 5.0
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        print(
            f'WARNING: chưa thấy logger subscribe {args.topic}; marker vẫn sẽ '
            'được publish nhưng hãy kiểm tra Terminal 4.', flush=True)
    else:
        print(f'Đã kết nối logger qua {args.topic}.', flush=True)
    print('Nhập marker rồi Enter; q sẽ yêu cầu logger lưu và thoát.', flush=True)

    try:
        for line in sys.stdin:
            command = line.strip()
            if not command:
                continue
            publisher.publish(String(data=command))
            rclpy.spin_once(node, timeout_sec=0.05)
            print(f'SENT: {command}', flush=True)
            if command == 'q':
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
