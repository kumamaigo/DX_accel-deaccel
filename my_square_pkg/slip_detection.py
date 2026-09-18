import csv
import math
import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# 駆動輪と従属輪の値の差を読み込みを行うやつ

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

    # gyro_odometry_publisher: encoder_driver.cpp の実装確認により、
    # 従動輪(後輪)エンコーダのみから計算されていることが判明済み。
    # -> こちらが「真の地面速度」に相当する。
    self.odom_sub = self.create_subscription(
        Odometry,
        '/aiformula_sensing/gyro_odometry_publisher/odom',
        self.odom_callback,
        qos_profile,
    )

    # 駆動輪(前輪)の実速度フィードバック。
    # motor_controller.py に追加した publisher (drive_wheel_feedback) を購読する。
    # ★CONFIRM★ 実際のnamespace(例: /aiformula_control/motor_controller/drive_wheel_feedback)
    # に合わせてトピック名を変更すること
    self.drive_feedback_sub = self.create_subscription(
        JointState,
        '/aiformula_control/motor_controller/drive_wheel_feedback',
        self.drive_feedback_callback,
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
    self.current_v_odom = 0.0    # 後輪(従動輪)ベース = 真の地面速度の推定値
    self.current_v_drive = 0.0   # 駆動輪(前輪)の実速度フィードバック = 空転の影響を受けうる
    self.target_v = 0.0
    self.state = "ACCEL1"  # 初期状態は第1加速
    self.phase_start_time = None

    self.slip_ratio_threshold = 0.15  # これを超えたら「空転あり」と判定する閾値(要調整)

    self.dt = 0.1
    self.timer = self.create_timer(self.dt, self.control_loop)

    # 6. データログ
    self.start_time = None
    self.time_log = []
    self.target_v_log = []
    self.ground_v_log = []        # 後輪(従動輪)ベース = 真の速度 (gyro_odometry_publisher)
    self.drive_v_log = []         # 駆動輪(前輪)の実速度フィードバック
    self.slip_ratio_log = []
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
    # gyro_odometry_publisher は従動輪(後輪)エンコーダのみから算出されているため、
    # 駆動輪の空転の影響を受けにくい「真の地面速度」に相当する。
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    self.current_v_odom = math.hypot(vx, vy)

  def drive_feedback_callback(self, msg):
    # motor_controller.py の drive_wheel_feedback をJointStateとして受信。
    # velocity[0], velocity[1] にそれぞれ左右駆動輪の地表速度[m/s]が入っている前提
    # (motor_controller.py 側で name=['drive_wheel_left','drive_wheel_right'] として publish)。
    if len(msg.velocity) < 2:
      return
    v_left, v_right = msg.velocity[0], msg.velocity[1]
    # 直進走行前提なので左右の平均を駆動輪速度とする
    self.current_v_drive = (abs(v_left) + abs(v_right)) / 2.0

  def calc_slip_ratio(self, v_drive, v_ground):
    """駆動輪速度と地面速度(後輪基準)からスリップ率を計算する。

    加速中: 駆動輪が地面より速く回る -> λ = (v_drive - v_ground) / v_drive
    減速中(ロック側): 地面より駆動輪が遅い -> λ = (v_ground - v_drive) / v_ground
    ここでは加速・減速どちらでも使える対称形の定義を採用。
    """
    denom = max(abs(v_drive), abs(v_ground), 1e-3)  # ゼロ割防止
    return (v_drive - v_ground) / denom

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
      # (後輪=真の地面速度基準。駆動輪が減速時にロックしていても
      #  後輪基準なら正しく「停止した」と判定できる)
      if self.target_v == 0.0 and self.current_v_odom <= 0.05:
        self.state = "DONE"

    # --- 【状態 6: 終了（保存処理）】 ---
    elif self.state == "DONE":
      self.target_v = 0.0
      if not self.graph_saved:
        self.save_and_plot_graph()
        self.graph_saved = True

    # スリップ率の計算 (駆動輪の実速度 vs 後輪=真の地面速度)
    slip_ratio = self.calc_slip_ratio(self.current_v_drive, self.current_v_odom)

    # データログの蓄積
    self.time_log.append(elapsed_total)
    self.target_v_log.append(self.target_v)
    self.ground_v_log.append(self.current_v_odom)
    self.drive_v_log.append(self.current_v_drive)
    self.slip_ratio_log.append(slip_ratio)

    self.get_logger().info(
        f"[{self.state}] 時間: {elapsed_total:.1f}s | 目標:"
        f" {self.target_v:.2f} m/s | 駆動輪(実測): {self.current_v_drive:.2f} m/s |"
        f" 後輪(真値): {self.current_v_odom:.2f} m/s | スリップ率: {slip_ratio:+.2f}"
    )

    if abs(slip_ratio) >= self.slip_ratio_threshold:
      self.get_logger().warn(
          f"【空転検出】 t={elapsed_total:.1f}s スリップ率={slip_ratio:+.2f}"
          f" (駆動輪 {self.current_v_drive:.2f} m/s vs 後輪 {self.current_v_odom:.2f} m/s)"
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
            "Drive_Wheel_Velocity[m/s]",
            "Ground_Velocity_RearWheel[m/s]",
            "Slip_Ratio",
        ])
        for t, v_target, v_drive, v_ground, slip in zip(
            self.time_log,
            self.target_v_log,
            self.drive_v_log,
            self.ground_v_log,
            self.slip_ratio_log,
        ):
          writer.writerow([
              f"{t:.3f}",
              f"{v_target:.3f}",
              f"{v_drive:.3f}",
              f"{v_ground:.3f}",
              f"{slip:.3f}",
          ])
      self.get_logger().info(
          f"【CSV保存完了】{csv_filename} にデータを保存しました。"
      )
    except Exception as e:
      self.get_logger().error(f"CSV保存失敗: {e}")

    # --- 2. グラフ描画・保存(上段:速度、下段:スリップ率) ---
    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(9, 7.5), sharex=True,
        gridspec_kw={"height_ratios": [2, 1]},
    )

    ax1.plot(
        self.time_log,
        self.target_v_log,
        label="Target Velocity",
        linestyle="--",
        color="blue",
    )
    ax1.plot(
        self.time_log,
        self.drive_v_log,
        label="Drive Wheel Velocity (front, may include slip)",
        color="red",
    )
    ax1.plot(
        self.time_log,
        self.ground_v_log,
        label="Ground Velocity (rear wheel, gyro_odometry_publisher)",
        color="green",
    )

    param_text = (
        f"[ Parameters ]\n"
        f"Target 1: 3.6 km/h ({self.v_max1:.2f} m/s)\n"
        f"Target 2: 5.2 km/h ({self.v_max2:.2f} m/s)\n"
        f"alpha: {self.alpha} | beta: {self.beta}\n"
        f"a_acc: {self.a_acc:.2f} m/s²\n"
        f"a_dec: {self.a_dec:.2f} m/s²"
    )

    ax1.text(
        0.97,
        0.05,
        param_text,
        transform=ax1.transAxes,
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

    ax1.set_title("Kobuki Two-Step Accel/Decel Response")
    ax1.set_ylabel("Velocity [m/s]")
    ax1.grid(True)
    ax1.legend(loc="upper left")

    # 下段: スリップ率 + 閾値ライン
    ax2.plot(
        self.time_log,
        self.slip_ratio_log,
        color="purple",
        label="Slip Ratio (drive vs ground)",
    )
    ax2.axhline(
        self.slip_ratio_threshold,
        color="black",
        linestyle=":",
        linewidth=1,
        label=f"Threshold (+{self.slip_ratio_threshold})",
    )
    ax2.axhline(
        -self.slip_ratio_threshold,
        color="black",
        linestyle=":",
        linewidth=1,
    )
    ax2.axhline(0.0, color="gray", linewidth=0.8)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Slip Ratio")
    ax2.grid(True)
    ax2.legend(loc="upper left")

    fig.tight_layout()

    filename = "two_step_accel_result.png"
    fig.savefig(filename)
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
