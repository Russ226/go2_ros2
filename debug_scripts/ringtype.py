
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class InspectCloud(Node):
    def __init__(self):
        super().__init__('inspect_utlidar_cloud')
        self.create_subscription(PointCloud2, '/utlidar/cloud', self.cb, 10)

    def cb(self, msg):
        rings = []
        times = []

        for offset in range(0, len(msg.data), msg.point_step):
            ring = struct.unpack_from('<H', msg.data, offset + 20)[0]
            point_time = struct.unpack_from('<f', msg.data, offset + 24)[0]

            rings.append(ring)
            times.append(point_time)

        unique_rings = sorted(set(rings))
        print(f'point_step: {msg.point_step}')
        print(f'ring min/max: {min(rings)} / {max(rings)}')
        print(f'unique ring count: {len(unique_rings)}')
        print(f'first 30 rings: {unique_rings[:30]}')
        print(f'last 30 rings: {unique_rings[-30:]}')
        print(f'time min/max: {min(times):.12f} / {max(times):.12f}')
        print(f'time span: {max(times) - min(times):.12f}')

        rclpy.shutdown()

rclpy.init()
node = InspectCloud()
rclpy.spin(node)
