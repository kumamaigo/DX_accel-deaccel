import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import matplotlib.pyplot as plt  # グラフ描画用
import csv                      # CSV出力用
import os

class KobukiAccelDecelTestNode(Node):
    def __init__(self):
        super().__init__('kobuki_accel_decel_test_node')

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

        # 物理パラメータ
        self.g = 9.81
        self.mu = 0.6
        self.h = 0.15
        self.Lf = 0.12
        self.Lr = 0.12
        self.L = self.Lf + self.Lr
        self.alpha = 1.0
        
        self.max_v_kmh = 2.5
        self.v_max = self.max_v_kmh / 3.6  # [m/s]

        # 手動で扱いやすい加減速度に設定（必要に応じて調整）
        self.a_acc = 0.3  # 加速度 [m/s^2]
        self.a_dec = 0.3  # 減速度 [m/s^2]

        self.current_v_odom = 0.0
        self.target_v = 0.0
        self.state = 'ACCEL'
        self.cruise_start_time = None

        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)

        # ----------------------------------------------------
        # グラフ＆CSV保存用のデータリスト＆時間管理
        # ----------------------------------------------------
        self.start_time = None
        self.time_log = []
        self.state_log = []
        self.target_v_log = []
        self.measured_v_log = []
        self.data_saved = False

        self.csv_filename = 'accel_deaccel_result.csv'
        self.png_filename = 'accel_deaccel_result.png'

    def odom_callback(self, msg):
        self.current_v_odom = msg.twist.twist.linear.x

    def control_loop(self):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        
        # 経過時間の計算（秒単位）
        elapsed_total = (now - self.start_time).nanoseconds / 1e9

        # --- 状態遷移 ---
        if self.state == 'ACCEL':
            self.target_v += self.a_acc * self.dt
            if self.target_v >= self.v_max:
                self.target_v = self.v_max
                self.state = 'CRUISE'
                self.cruise_start_time = now

        elif self.state == 'CRUISE':
            elapsed_cruise = (now - self.cruise_start_time).nanoseconds / 1e9
            if elapsed_cruise >= 3.0:  # 3秒間定速走行
                self.state = 'DECEL'

        elif self.state == 'DECEL':
            self.target_v -= self.a_dec * self.dt
            if self.target_v <= 0.0:
                self.target_v = 0.0
                self.state = 'DONE'

        elif self.state == 'DONE':
            self.target_v = 0.0
            if not self.data_saved:
                self.save_data_and_plot()
                self.data_saved = True

        # データログへの追加
        self.time_log.append(elapsed_total)
        self.state_log.append(self.state)
        self.target_v_log.append(self.target_v)
        self.measured_v_log.append(self.current_v_odom)

        # ログ出力
        self.get_logger().info(
            f"[{self.state}] 時間: {elapsed_total:.1f}s | 目標: {self.target_v:.2f} m/s | 実測: {self.current_v_odom:.2f} m/s"
        )

        # 送信
        twist = Twist()
        twist.linear.x = float(self.target_v)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def save_data_and_plot(self):
        """ CSV出力およびグラフプロットの実行 """
        self.export_to_csv()
        self.plot_graph()

    def export_to_csv(self):
        """ 実験データをCSV形式で出力 """
        try:
            with open(self.csv_filename, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                # ヘッダー書き込み
                writer.writerow(['time_sec', 'state', 'target_velocity_mps', 'measured_velocity_mps'])
                # データ行の書き込み
                for t, st, v_tgt, v_meas in zip(self.time_log, self.state_log, self.target_v_log, self.measured_v_log):
                    writer.writerow([f"{t:.3f}", st, f"{v_tgt:.4f}", f"{v_meas:.4f}"])
            self.get_logger().info(f"【CSV保存完了】データファイルを {self.csv_filename} に保存しました。")
        except Exception as e:
            self.get_logger().error(f"CSV保存中にエラーが発生しました: {e}")

    def plot_graph(self):
        """ 実験データのプロットと保存 """
        plt.figure(figsize=(9, 5))
        plt.plot(self.time_log, self.target_v_log, label='Target Velocity (m/s)', linestyle='--', color='blue')
        plt.plot(self.time_log, self.measured_v_log, label='Measured Velocity (Odom)', color='red')
        
        plt.title('Kobuki Velocity Response (Accel / Cruise / Decel)')
        plt.xlabel('Time [s]')
        plt.ylabel('Velocity [m/s]')
        plt.grid(True)
        plt.legend()
        
        # 画像ファイルとして保存
        plt.savefig(self.png_filename)
        self.get_logger().info(f"【グラフ保存完了】{self.png_filename} に画像を保存しました。")
        plt.show()

    def emergency_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = KobukiAccelDecelTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.emergency_stop()
        if not node.data_saved and len(node.time_log) > 0:
            node.save_data_and_plot()
            node.data_saved = True
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()