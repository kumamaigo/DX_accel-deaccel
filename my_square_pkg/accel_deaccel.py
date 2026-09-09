import csv
import math
import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class KobukiTwoStepAccelTestNode(Node):

  def __init__(self):
    super().__init__('kobuki_two_step_accel_node')

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

    # --- 追従性を高めるための安全係数設定 ---
    self.alpha = 0.8  # 加速用の安全係数 (青破線の傾きを緩やかにして追従させる)
    self.beta = 0.5  # 減速用の安全係数 (急ブレーキによる遅れを防ぐ)

    # 2段階の目標速度設定
    self.v_max1 = 3.6 / 3.6  # 第1目標: 3.6 km/h = 1.0 m/s
    self.v_max2 = 7.2 / 3.6  # 第2目標: 7.2 km/h ≒ 2m/s

    # 各定速フェーズの時間設定
    self.cruise1_time = 0.8  # 第1定速時間 [s]
    self.cruise2_time = 0.8  # 第2定速時間 [s]

    # 4. 理論限界計算 ＋ 安全係数の適用
    raw_a_acc = self.calc_accel_limit()
    raw_a_dec = self.calc_decel_limit()

    self.a_acc = raw_a_acc * self.alpha
    self.a_dec = raw_a_dec * self.beta

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
    self.state = "ACCEL1"  # 初期状態は第1加速
    self.phase_start_time = None

    self.dt = 0.1
    self.timer = self.create_timer(self.dt, self.control_loop)

    # 6. データログ
    self.start_time = None
    self.time_log = []
    self.target_v_log = []
    self.measured_v_log = []
    self.graph_saved = False

  def calc_accel_limit(self):
    term1 = (self.mu * self.g * self.Lr) / (self.L + self.mu * self.h)
    term2 = self.g * (self.Lr / self.h)
    term3 = self.mu * self.g
    return min(term1, term2, term3)

  def calc_decel_limit(self):
    term1 = (self.mu * self.g * self.Lr) / (self.L - self.mu * self.h)
    term2 = self.g * (self.Lf / self.h)
    term3 = self.mu * self.g
    return min(term1, term2, term3)

  def odom_callback(self, msg):
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    self.current_v_odom = math.hypot(vx, vy)

  def control_loop(self):
    now = self.get_clock().now()
    if self.start_time is None:
      self.start_time = now

    elapsed_total = (now - self.start_time).nanoseconds / 1e9

    # --- 【状態 1: 第1加速フェーズ (0 -> 3.6km/h)】 ---
    if self.state == "ACCEL1":
      self.target_v += self.a_acc * self.dt
      if self.target_v >= self.v_max1:
        self.target_v = self.v_max1
        self.state = "CRUISE1"
        self.phase_start_time = now

    # --- 【状態 2: 第1定速フェーズ (3.6km/h で 0.5秒間)】 ---
    elif self.state == "CRUISE1":
      elapsed_phase = (now - self.phase_start_time).nanoseconds / 1e9
      if elapsed_phase >= self.cruise1_time:
        self.state = "ACCEL2"

    # --- 【状態 3: 第2加速フェーズ (3.6km/h -> 5.2km/h)】 ---
    elif self.state == "ACCEL2":
      self.target_v += self.a_acc * self.dt
      if self.target_v >= self.v_max2:
        self.target_v = self.v_max2
        self.state = "CRUISE2"
        self.phase_start_time = now

    # --- 【状態 4: 第2定速フェーズ (5.2km/h で 0.5秒間)】 ---
    elif self.state == "CRUISE2":
      elapsed_phase = (now - self.phase_start_time).nanoseconds / 1e9
      if elapsed_phase >= self.cruise2_time:
        self.state = "DECEL"

    # --- 【状態 5: 減速フェーズ (5.2km/h -> 0km/h)】 ---
    elif self.state == "DECEL":
      self.target_v -= self.a_dec * self.dt
      if self.target_v <= 0.0:
        self.target_v = 0.0

      # 目標速度が0かつ、実測速度も十分に停止するまで待機
      if self.target_v == 0.0 and self.current_v_odom <= 0.05:
        self.state = "DONE"

    # --- 【状態 6: 終了（保存処理）】 ---
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

    # 速度コマンド送信
    twist = Twist()
    twist.linear.x = float(self.target_v)
    twist.angular.z = 0.0
    self.cmd_pub.publish(twist)

  def save_and_plot_graph(self):
    # --- 1. CSV書き出し ---
    csv_filename = "two_step_accel_result.csv"
    try:
      with open(csv_filename, mode="w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["# --- Experiment Parameters ---"])
        writer.writerow(["# Friction coefficient (mu)", self.mu])
        writer.writerow(["# Target Speed 1 (km/h)", 3.6])
        writer.writerow(["# Target Speed 2 (km/h)", 5.2])
        writer.writerow(["# Accel Safety Factor (alpha)", self.alpha])
        writer.writerow(["# Decel Safety Factor (beta)", self.beta])
        writer.writerow(
            ["# Applied Accel Limit (a_acc [m/s^2])", f"{self.a_acc:.3f}"]
        )
        writer.writerow(
            ["# Applied Decel Limit (a_dec [m/s^2])", f"{self.a_dec:.3f}"]
        )
        writer.writerow([])

        writer.writerow([
            "Time[s]",
            "Target_Velocity[m/s]",
            "Measured_Velocity[m/s]",
        ])
        for t, v_target, v_meas in zip(
            self.time_log, self.target_v_log, self.measured_v_log
        ):
          writer.writerow([f"{t:.3f}", f"{v_target:.3f}", f"{v_meas:.3f}"])
      self.get_logger().info(
          f"【CSV保存完了】{csv_filename} にデータを保存しました。"
      )
    except Exception as e:
      self.get_logger().error(f"CSV保存失敗: {e}")

    # --- 2. グラフ描画・保存 ---
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

    param_text = (
        f"[ Parameters ]\n"
        f"Target 1: 3.6 km/h ({self.v_max1:.2f} m/s)\n"
        f"Target 2: 5.2 km/h ({self.v_max2:.2f} m/s)\n"
        f"alpha: {self.alpha} | beta: {self.beta}\n"
        f"a_acc: {self.a_acc:.2f} m/s²\n"
        f"a_dec: {self.a_dec:.2f} m/s²"
    )

    plt.gca().text(
        0.97,
        0.05,
        param_text,
        transform=plt.gca().transAxes,
        fontsize=9,
        verticalalignment="bottom",
        horizontalalignment="right",
        bbox=dict(
            boxstyle="round,pad=0.5",
            facecolor="white",
            alpha=0.8,
            edgecolor="gray",
        ),
    )

    plt.title("Kobuki Two-Step Accel/Decel Response")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.grid(True)
    plt.legend(loc="upper left")

    filename = "two_step_accel_result.png"
    plt.savefig(filename)
    self.get_logger().info(
        f"【グラフ保存完了】{filename} に画像を保存しました。"
    )
    plt.show()

  def emergency_stop(self):
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    self.cmd_pub.publish(twist)
    self.get_logger().warn("【緊急停止】速度 0 を送信しました。")


def main(args=None):
  rclpy.init(args=args)
  node = KobukiTwoStepAccelTestNode()

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