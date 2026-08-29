import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math

class SquareMoveNode(Node):
    def __init__(self):
        super().__init__('square_move_node')

        self.cmd_pub = self.create_publisher(Twist, '/commands/velocity', 10)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )

        self.x = None
        self.y = None
        self.yaw = None

        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.state = 'INIT'
        self.sides_completed = 0
        self.target_distance = 0.6  # 1辺 1.5m
        self.target_angle = math.pi / 2.0  # 90度

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        if self.x is None or self.y is None or self.yaw is None:
            return

        twist = Twist()

        if self.state == 'INIT':
            self.start_x = self.x
            self.start_y = self.y
            self.state = 'FORWARD'
            self.get_logger().info(f"--- 辺 {self.sides_completed + 1} 前進開始 (1.5m) ---")

        elif self.state == 'FORWARD':
            dist = math.sqrt((self.x - self.start_x)**2 + (self.y - self.start_y)**2)
            
            if dist < self.target_distance:
                twist.linear.x = 0.15
            else:
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)
                self.start_yaw = self.yaw
                self.state = 'TURN'
                self.get_logger().info("--- 90度 右回転開始 ---")

        elif self.state == 'TURN':
            angle_diff = abs(self.normalize_angle(self.yaw - self.start_yaw))

            if angle_diff < self.target_angle:
                twist.angular.z = -0.3
            else:
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.sides_completed += 1

                if self.sides_completed < 4:
                    self.start_x = self.x
                    self.start_y = self.y
                    self.state = 'FORWARD'
                    self.get_logger().info(f"--- 辺 {self.sides_completed + 1} 前進開始 (1.5m) ---")
                else:
                    self.state = 'DONE'
                    self.get_logger().info("正方形移動（1.5m x 4辺）完了！")

        elif self.state == 'DONE':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SquareMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
