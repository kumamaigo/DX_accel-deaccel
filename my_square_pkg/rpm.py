import struct
from can_msgs.msg import Frame
import rclpy
from rclpy.node import Node


class RpmMonitorNode(Node):

  def __init__(self):
    super().__init__('rpm_monitor_node')
    self.subscription = self.create_subscription(
        Frame, 'sub_can', self.can_callback, 10
    )
    self.get_logger().info(
        '【RPMモニター起動】CAN ID 0x211 のデータを待機中...'
    )

  def can_callback(self, msg):
    if msg.id == 0x211 and len(msg.data) >= 8:
      rpm_l, rpm_r = struct.unpack('<ii', bytes(msg.data[:8]))
      diff = abs(rpm_l - rpm_r)
      self.get_logger().info(
          f'左輪: {rpm_l:5d} RPM | 右輪: {rpm_r:5d} RPM | 左右差: {diff:5d}'
      )


def main(args=None):
  rclpy.init(args=args)
  node = RpmMonitorNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()