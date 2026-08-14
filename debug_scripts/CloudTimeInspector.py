import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class CloudTimeInspector(Node):
    def __init__(self):
        super().__init__('cloud_time_inspector')
        self.sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud', self.cb, 10)

    def cb(self, msg):
        field = next((f for f in msg.fields if f.name == 'time'), None)
        if field is None:
            self.get_logger().error('No time field')
            return

        values = [
            struct.unpack_from('<f', msg.data, i * msg.point_step + field.offset)[0]
            for i in range(msg.width * msg.height)
        ]

        self.get_logger().info(
            f'points={len(values)} time=[{min(values):.9f}, '
            f'{max(values):.9f}] span={max(values)-min(values):.9f} '
            f'first={values[:8]}')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = CloudTimeInspector()
    rclpy.spin(node)

if __name__ == '__main__':
    main()