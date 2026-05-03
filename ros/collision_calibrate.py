#!/usr/bin/env python3
"""
Collision calibration logger — continuously logs min ranges from lidar + ToF.

Writes JSON-lines to /workspace/logs/collision_cal.jsonl with rolling min
ranges every 0.5s. Drive the robot into walls; approaches are auto-detected
when min range dips below 0.5m then recovers.

Subscribes to:
  /scan           (front lidar)
  /tof_rear_left  (rear left ToF)
  /tof_rear_right (rear right ToF)

Usage (from host, background):
  docker compose exec -T test-drive bash -c \
    "source /opt/ros/jazzy/setup.bash && python3 /workspace/ros/collision_calibrate.py"

Kill with Ctrl+C or docker compose exec -T test-drive pkill -f collision_calibrate
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import json
import threading
import time
import sys


LOG_PATH = '/workspace/logs/collision_cal.jsonl'
APPROACH_RESET_THRESHOLD = 0.5  # reset approach tracking when min > this


class CollisionCalibrator(Node):
    def __init__(self):
        super().__init__('collision_calibrator')

        self.lock = threading.Lock()

        # Latest instantaneous min ranges
        self.scan_latest = float('inf')
        self.tof_left_latest = float('inf')
        self.tof_right_latest = float('inf')

        # Per-approach minimums (reset when robot backs away)
        self.approach_scan_min = float('inf')
        self.approach_tof_left_min = float('inf')
        self.approach_tof_right_min = float('inf')
        self.approach_active = False

        # Detected approach events (min range dipped below threshold then recovered)
        self.approaches = []

        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/tof_rear_left', self.tof_left_cb, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/tof_rear_right', self.tof_right_cb, qos_profile_sensor_data)

        # Log timer (2 Hz)
        self.create_timer(0.5, self.log_cb)

        # Open log file
        self.log_file = open(LOG_PATH, 'w')
        self.get_logger().info(f'Collision calibrator logging to {LOG_PATH}')
        self.get_logger().info('Drive into walls! Approaches auto-detected when min < 0.5m.')

    def _min_valid_range(self, msg):
        min_r = float('inf')
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max and r < min_r:
                min_r = r
        return min_r

    def scan_cb(self, msg):
        val = self._min_valid_range(msg)
        with self.lock:
            self.scan_latest = val

    def tof_left_cb(self, msg):
        val = self._min_valid_range(msg)
        with self.lock:
            self.tof_left_latest = val

    def tof_right_cb(self, msg):
        val = self._min_valid_range(msg)
        with self.lock:
            self.tof_right_latest = val

    def log_cb(self):
        with self.lock:
            scan = self.scan_latest
            tl = self.tof_left_latest
            tr = self.tof_right_latest

        overall_min = min(scan, tl, tr)

        # Track approaches: robot getting close then backing away
        if overall_min < APPROACH_RESET_THRESHOLD:
            if not self.approach_active:
                self.approach_active = True
                self.approach_scan_min = float('inf')
                self.approach_tof_left_min = float('inf')
                self.approach_tof_right_min = float('inf')

            self.approach_scan_min = min(self.approach_scan_min, scan)
            self.approach_tof_left_min = min(self.approach_tof_left_min, tl)
            self.approach_tof_right_min = min(self.approach_tof_right_min, tr)

        elif self.approach_active and overall_min > APPROACH_RESET_THRESHOLD:
            approach = {
                'n': len(self.approaches) + 1,
                'time': time.strftime('%H:%M:%S'),
                'scan_min': round(self.approach_scan_min, 4) if self.approach_scan_min < 100 else None,
                'tof_left_min': round(self.approach_tof_left_min, 4) if self.approach_tof_left_min < 100 else None,
                'tof_right_min': round(self.approach_tof_right_min, 4) if self.approach_tof_right_min < 100 else None,
            }
            self.approaches.append(approach)
            self.get_logger().info(
                f'Approach #{approach["n"]}: scan={approach["scan_min"]}, '
                f'tof_L={approach["tof_left_min"]}, tof_R={approach["tof_right_min"]}')
            self.approach_active = False

        # Write log line
        def clean(v):
            return round(v, 4) if v < 100 else None

        entry = {
            't': round(time.time(), 2),
            'scan': clean(scan),
            'tof_L': clean(tl),
            'tof_R': clean(tr),
            'approaching': self.approach_active,
        }
        self.log_file.write(json.dumps(entry) + '\n')
        self.log_file.flush()

        # Status to stderr
        def fmt(v):
            return f'{v:.3f}' if v < 100 else ' --- '

        status = 'CLOSE' if self.approach_active else '     '
        sys.stderr.write(
            f'\r  scan:{fmt(scan)}  tof_L:{fmt(tl)}  tof_R:{fmt(tr)}  '
            f'approaches:{len(self.approaches)}  [{status}]')
        sys.stderr.flush()

    def destroy_node(self):
        if self.approaches:
            self.log_file.write(json.dumps({'summary': self.approaches}) + '\n')
        self.log_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CollisionCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stderr.write('\n')
        if node.approaches:
            sys.stderr.write(f'\n=== {len(node.approaches)} approaches detected ===\n')
            for a in node.approaches:
                sys.stderr.write(f'  #{a["n"]}: scan={a["scan_min"]}, '
                                 f'tof_L={a["tof_left_min"]}, tof_R={a["tof_right_min"]}\n')
        print(json.dumps({'approaches': node.approaches}, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
