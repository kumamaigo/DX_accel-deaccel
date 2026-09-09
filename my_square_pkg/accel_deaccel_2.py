import math
import csv
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy



class KobukiAccelDecelTestNode(Node):

  def __init__(self):
    super().__init__('kobuki_accel_decel_test_node')

    # 1. Publisher
    self.cmd_pub = self.create_publisher(
        Twist, '/aiformula_control/twist_mux/cmd_vel', 10
    )

    # 2. Subscriber
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        depth=10,
    )
    self.odom_sub = self.create_subscription(
        Odometry,
        '/aiformula_sensing/gyro_odometry_publisher/odom',
        self.odom_callback,
        qos_profile,
    )

    # 3. 物理パラメータ
    self.g = 9.81  # 重力加速度 [m/s^2]
    self.mu = 0.35  # 路面の摩擦係数
    self.h = 0.15  # 重心の高さ [m]
    self.Lf = 0.15  # 前軸から重心までの距離 [m]
    self.Lr = 0.7  # 後軸から重心までの距離 [m]
    self.L = self.Lf + self.Lr  # ホイールベース [m]

    # --- 【追加】加減速で独立した安全係数 ---
    self.alpha = 1.0  # 加速用の安全係数 (1.0 = 理論値100%)
    self.beta = 0.4  # 減速用の安全係数 (追従遅れを防ぐため 0.4 など小さめに設定)    

    # 最高速度の設定 (時速3.6km = 約1.0m/s)
    self.max_v_kmh = 3.6
    self.v_max = self.max_v_kmh / 3.6  # [m/s] 単位に換算

    # 通常速度の時間
    self.normal_vel_time = 3.0  #[s]

    # 4. 理論限界計算 ＋ 独立した安全係数の適用
    raw_a_acc = self.calc_accel_limit()
    raw_a_dec = self.calc_decel_limit()


    # 安全係数を加速には alpha、減速には beta を適用（raw_a_acc,dccの詳細は下記に）
    self.a_acc = raw_a_acc * self.alpha
    self.a_dec = raw_a_dec * self.beta

    # ターミナルのログ表示(デバック用)
    self.get_logger().info(
        f"【理論限界】加速: {raw_a_acc:.2f} m/s^2 | 減速: {raw_a_dec:.2f} m/s^2"
    )
    self.get_logger().info(
        f"【安全適用】加速(α={self.alpha}): {self.a_acc:.2f} m/s^2 |"
        f" 減速(β={self.beta}): {self.a_dec:.2f} m/s^2"
    )

    # 5. 制御ループ用の変数
    self.current_v_odom = 0.0
    self.target_v = 0.0
    self.state = "ACCEL"
    self.cruise_start_time = None

    self.dt = 0.1
    self.timer = self.create_timer(self.dt, self.control_loop)

    # 6. データログ
    self.start_time = None
    self.time_log = []
    self.target_v_log = []
    self.measured_v_log = []
    self.graph_saved = False

# --------------------------------------------------------------------
#  加速理論値計算
  def calc_accel_limit(self):
    term1 = (self.mu * self.g * self.Lr) / (self.L + self.mu * self.h)
    term2 = self.g * (self.Lr / self.h)
    term3 = self.mu * self.g
    return min(term1, term2, term3)

#　減速理論値計算
  def calc_decel_limit(self):
    term1 = (self.mu * self.g * self.Lr) / (self.L - self.mu * self.h)
    term2 = self.g * (self.Lf / self.h)
    term3 = self.mu * self.g
    return min(term1, term2, term3)  
# --------------------------------------------------------------------

# オドメトリの読み込み作業
  def odom_callback(self, msg):
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    self.current_v_odom = math.hypot(vx, vy)

# 正確な経過時間を計測
  def control_loop(self):
    now = self.get_clock().now()
    if self.start_time is None:
      self.start_time = now

    elapsed_total = (now - self.start_time).nanoseconds / 1e9

    # --- 【状態 1: 加速フェーズ】 ---
    if self.state == "ACCEL":
      self.target_v += self.a_acc * self.dt
      if self.target_v >= self.v_max:
        self.target_v = self.v_max
        self.state = "CRUISE"
        self.cruise_start_time = now

    # --- 【状態 2: 定速走行フェーズ（3秒間）】 ---
    elif self.state == "CRUISE":
      elapsed_cruise = (now - self.cruise_start_time).nanoseconds / 1e9
      if elapsed_cruise >= self.normal_vel_time:
        self.state = "DECEL"

    # --- 【状態 3: 減速フェーズ】 ---
    elif self.state == "DECEL":
      self.target_v -= self.a_dec * self.dt
      if self.target_v <= 0.0:
        self.target_v = 0.0

      # 【変更】目標速度が0かつ、実測速度も十分に落ちる（停止）までDONEに移行しない
      if self.target_v == 0.0 and self.current_v_odom <= 0.05:
        self.state = "DONE"

    # --- 【状態 4: 終了（グラフ保存）】 ---
    elif self.state == "DONE":
      self.target_v = 0.0
      if not self.graph_saved:
        self.save_and_plot_graph()
        self.graph_saved = True

    # データログの蓄積
    self.time_log.append(elapsed_total)
    self.target_v_log.append(self.target_v)
    self.measured_v_log.append(self.current_v_odom)

    self.get_logger().info(
        f"[{self.state}] 時間: {elapsed_total:.1f}s | 目標:"
        f" {self.target_v:.2f} m/s | 実測: {self.current_v_odom:.2f} m/s"
    )

    # 送信
    twist = Twist()
    twist.linear.x = float(self.target_v)
    twist.angular.z = 0.0
    self.cmd_pub.publish(twist)

# -----------実験結果の保存------------
  def save_and_plot_graph(self):
    plt.figure(figsize=(9, 5))
    plt.plot(
        self.time_log,
        self.target_v_log,
        label="Target Velocity (m/s)",
        linestyle="--",
        color="blue",
    )
    plt.plot(
        self.time_log,
        self.measured_v_log,
        label="Measured Velocity (Odom)",
        color="red",
    )

    plt.title("Kobuki Kinematic Accel/Decel Response (With Safety Factors)")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.grid(True)
    plt.legend()

    filename = "accel_deaccel_result.png"
    plt.savefig(filename)
    self.get_logger().info(
        f"【グラフ保存完了】{filename} に画像を保存しました。"
    )
    plt.show()


  def emergency_stop(self):
    """Ctrl+C などの割り込み時にロボットへ即座に速度0を送る関数"""
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    self.cmd_pub.publish(twist)
    self.get_logger().warn("【緊急停止】速度 0 を送信しました。")


def main(args=None):
  rclpy.init(args=args)
  node = KobukiAccelDecelTestNode()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.emergency_stop()
    if not node.graph_saved and len(node.time_log) > 0:
      node.save_and_plot_graph()
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == "__main__":
  main()