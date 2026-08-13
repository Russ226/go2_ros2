#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ClosureDrift(Node):
    def __init__(self, odom_frame, base_frame, output):
        super().__init__('measure_odom_closure_drift')
        self.odom_frame = odom_frame.lstrip('/')
        self.base_frame = base_frame.lstrip('/')
        self.output = output
        self.start = None
        self.latest = None
        self.rows = []
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 100)

    def tf_callback(self, msg):
        for t in msg.transforms:
            parent = t.header.frame_id.lstrip('/')
            child = t.child_frame_id.lstrip('/')
            if parent != self.odom_frame or child != self.base_frame:
                continue
            stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            pose = (
                stamp,
                t.transform.translation.x,
                t.transform.translation.y,
                yaw_from_quaternion(t.transform.rotation),
            )
            if self.start is None:
                self.start = pose
                self.get_logger().info(
                    f'Start captured: x={pose[1]:.3f}, y={pose[2]:.3f}, yaw={math.degrees(pose[3]):.2f} deg'
                )
            self.latest = pose
            self.rows.append(pose)
            return

    def report(self):
        if self.start is None or self.latest is None:
            self.get_logger().error('No odom -> base_link transform received.')
            return
        _, x0, y0, yaw0 = self.start
        _, x1, y1, yaw1 = self.latest
        dx_world, dy_world = x1 - x0, y1 - y0
        dx_body = math.cos(yaw0) * dx_world + math.sin(yaw0) * dy_world
        dy_body = -math.sin(yaw0) * dx_world + math.cos(yaw0) * dy_world
        dyaw = wrap(yaw1 - yaw0)
        distance = math.hypot(dx_world, dy_world)
        print('\n=== Odom closure drift ===')
        print(f'Start:             x={x0:.3f} m, y={y0:.3f} m, yaw={math.degrees(yaw0):.2f} deg')
        print(f'End:               x={x1:.3f} m, y={y1:.3f} m, yaw={math.degrees(yaw1):.2f} deg')
        print(f'Forward error:     {dx_body:.3f} m')
        print(f'Lateral error:     {dy_body:.3f} m')
        print(f'Translation error: {distance:.3f} m')
        print(f'Heading error:     {math.degrees(dyaw):.2f} deg')
        with open(self.output, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['stamp_s', 'x_m', 'y_m', 'yaw_rad'])
            writer.writerows(self.rows)
        print(f'Trajectory samples: {len(self.rows)}')
        print(f'Wrote: {self.output}')


def main():
    parser = argparse.ArgumentParser(description='Measure odometry closure error from /tf.')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--output', default='odom_closure_trajectory.csv')
    args = parser.parse_args()
    rclpy.init()
    node = ClosureDrift(args.odom_frame, args.base_frame, Path(args.output).expanduser())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
