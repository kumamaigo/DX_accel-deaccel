import csv
import os
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class KobukiJerkControlNode(Node):

  def __init__(self):
    super().__init__('kobuki_jerk_control_node')

    # ROS 2 通信設定
    self.cmd_pub = self.create_publisher(
        Twist, '/aiformula_control/twist_mux/cmd_vel', 10
    )

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

    # ----------------------------------------------------
    # 1. 物理パラメータ設定
    # ----------------------------------------------------
    self.g = 9.81  # 重力加速度 [m/s^2]
    self.mu = 0.25  # 路面（フローリング）の摩擦係数
    self.beta = 0.3  # 加速安全率
    self.alpha = 1.0  # 減速安全率
    self.h = 0.19  # 重心高さ [m]
    self.Lf = 0.15  # 前軸〜重心距離 [m]
    self.Lr = 0.70  # 後軸〜重心距離 [m]
    self.L = self.Lf + self.Lr  # ホイールベース [m]

    self.max_v_kmh = 2.5  # 最高速度 [km/h]
    self.v_max = self.max_v_kmh / 3.6  # 最高速度 [m/s] (約 0.694 m/s)

    # 加速度・減速度の上限値を計算 [m/s^2]
    self.a_acc_max = self.calc_accel_limit()
    self.a_dec_max = self.calc_decel_limit()

    # ----------------------------------------------------
    # 2. ジャーク（躍度）の設定 [m/s^3]
    # ----------------------------------------------------
    # 1秒間にどれだけ加速度を変化させて良いかを表す指標。
    # 値が小さいほどなめらか（S字が緩やか）になり、大きいほど台形制御に近づきます。
    self.jerk_max = 0.5  # [m/s^3]

    # 現在のリアルタイムな「加速度」と「目標速度」の保持変数
    self.current_a = 0.0  # 現在の加速度 [m/s^2]
    self.target_v = 0.0    # 現在の目標速度 [m/s]
    self.current_v_odom = 0.0  # 実測速度 [m/s]

    self.state = 'ACCEL'
    self.cruise_start_time = None

    # タイマー周期
    self.dt = 0.1
    self.timer = self.create_timer(self.dt, self.control_loop)

    # ログ・グラフ記録用変数
    self.start_time = None
    self.last_control_time = None
    self.time_log = []
    self.state_log = []
    self.target_v_log = []
    self.measured_v_log = []
    self.accel_log = []  # 加速度の変化も記録
    self.data_saved = False

    self.csv_filename = 'jerk_control_result.csv'
    self.png_filename = 'jerk_control_result.png'

  def calc_accel_limit(self):
    """限界加速力を計算 [m/s^2]"""
    a_acc = (self.mu * self.g * self.Lr) / (self.L + self.mu * self.h)
    return float(a_acc * self.beta)

  def calc_decel_limit(self):
    """限界減速力を計算 [m/s^2]"""
    a_dec = (
        (self.mu * self.g * self.Lf) / (self.L - self.mu * self.h)
    ) * self.alpha
    return float(a_dec)

  def odom_callback(self, msg):
    self.current_v_odom = msg.twist.twist.linear.x

  def control_loop(self):
    now = self.get_clock().now()

    # 初回呼び出し時の時刻設定
    if self.start_time is None:
      self.start_time = now
      self.last_control_time = now
      return

    # 実測経過時間 (actual_dt) の算定
    actual_dt = (now - self.last_control_time).nanoseconds / 1e9
    self.last_control_time = now
    if actual_dt <= 0.0:
      actual_dt = self.dt

    elapsed_total = (now - self.start_time).nanoseconds / 1e9

    # ----------------------------------------------------
    # 3. ジャーク制御（S字加速・減速）アルゴリズム
    # 現在のリアルタイムな「加速度」と「目標速度」の保持変数
    # self.current_a   : 現在の加速度 [m/s^2]
    # self.target_v = 0.0   : 現在の目標速度 [m/s]
    # self.current_v_odom = 0.0 : 実測速度 [m/s]
    # ----------------------------------------------------
    
    if self.state == 'ACCEL':
      # 残りの到達目標速度までの差分を示している(目標スピードまで、あとどれくらい離れているか？)
      v_diff = self.v_max - self.target_v

      # 【重要】現在の加速度(current_a)を0まで安全に減衰させるために必要な速度変化量
      #  戻す間に勝手に伸びるスピードのこと
      # 式: Delta_V = a^2 / (2 * Jerk)
      # jerk_maxの値は50行あたりに記述
      v_ramp_down = (self.current_a**2) / (2.0 * self.jerk_max)

      # --- 判定1: 目標速度が近づいたら加速度を下げ始める（S字の上側のカーブ） ---
      # 【今すぐアクセルを緩め始めないと、目標速度をオーバーしてしまう！」という警告】

      if v_diff <= v_ramp_down: # 現在の速度とジャーク速度を見ている
        self.current_a -= self.jerk_max * actual_dt
        if self.current_a < 0.0:
          self.current_a = 0.0

      # --- 判定2: 加速度が上限に達していない場合、じわじわ加速度を上げる（S字の下側のカーブ） ---
      # 【発進時の衝撃を無くすため、アクセルを少しずつ深く踏み込んでいる最中】
      
      elif self.current_a < self.a_acc_max:     # 現在の速度とジャーク速度を見ている
        self.current_a += self.jerk_max * actual_dt
        if self.current_a > self.a_acc_max:
          self.current_a = self.a_acc_max

      # --- 判定3: 加速度が上限（a_acc_max）に達している場合、一定の加速度を維持 ---
      else:
        self.current_a = self.a_acc_max

      # 計算した加速度で目標速度を更新 (v = v + a * dt)
      self.target_v += self.current_a * actual_dt

      # 目標速度(v_max)に到達し、加速度がほぼ0になったら定速走行(CRUISE)へ
      if self.target_v >= self.v_max and self.current_a <= 0.01:
        self.target_v = self.v_max
        self.current_a = 0.0
        self.state = 'CRUISE'
        self.cruise_start_time = now

    elif self.state == 'CRUISE':
      self.current_a = 0.0
      elapsed_cruise = (now - self.cruise_start_time).nanoseconds / 1e9
      if elapsed_cruise >= 3.0:  # 3秒間定速走行
        self.state = 'DECEL'

    elif self.state == 'DECEL':
      # 残りの完全停止(0 m/s)までの速度差分
      v_diff = self.target_v - 0.0

      # 減速度を0に引き戻すために必要な速度変化量
      v_ramp_down = (self.current_a**2) / (2.0 * self.jerk_max)

      # --- 判定1: 停止間近になったら減速度を弱める（スムーズな停止） ---
      if v_diff <= v_ramp_down:
        self.current_a -= self.jerk_max * actual_dt
        if self.current_a < 0.0:
          self.current_a = 0.0

      # --- 判定2: 減速度を上限に向けて上げていく ---
      elif self.current_a < self.a_dec_max:
        self.current_a += self.jerk_max * actual_dt
        if self.current_a > self.a_dec_max:
          self.current_a = self.a_dec_max

      # --- 判定3: 最大減速度を維持 ---
      else:
        self.current_a = self.a_dec_max

      # 減速処理（目標速度から引く）
      self.target_v -= self.current_a * actual_dt

      # 速度が0になり、減速度も0になったら完了
      if self.target_v <= 0.0 and self.current_a <= 0.01:
        self.target_v = 0.0
        self.current_a = 0.0
        self.state = 'DONE'

    elif self.state == 'DONE':
      self.target_v = 0.0
      self.current_a = 0.0
      if not self.data_saved:
        self.save_data_and_plot()
        self.data_saved = True

    # ログ記録
    self.time_log.append(elapsed_total)
    self.state_log.append(self.state)
    self.target_v_log.append(self.target_v)
    self.measured_v_log.append(self.current_v_odom)
    self.accel_log.append(self.current_a)

    self.get_logger().info(
        f'[{self.state}] 時間: {elapsed_total:.1f}s | 目標速:'
        f' {self.target_v:.2f}m/s | 加速: {self.current_a:.2f}m/s^2 | 実測:'
        f' {self.current_v_odom:.2f}m/s'
    )

    # 速度命令の送信
    twist = Twist()
    twist.linear.x = float(self.target_v)
    twist.angular.z = 0.0
    self.cmd_pub.publish(twist)

  def save_data_and_plot(self):
    self.export_to_csv()
    self.plot_graph()

  def export_to_csv(self):
    try:
      with open(
          self.csv_filename, mode='w', newline='', encoding='utf-8'
      ) as f:
        writer = csv.writer(f)
        writer.writerow([
            'time_sec',
            'state',
            'target_velocity_mps',
            'measured_velocity_mps',
            'current_accel_mps2',
        ])
        for t, st, v_tgt, v_meas, acc in zip(
            self.time_log,
            self.state_log,
            self.target_v_log,
            self.measured_v_log,
            self.accel_log,
        ):
          writer.writerow(
              [f'{t:.3f}', st, f'{v_tgt:.4f}', f'{v_meas:.4f}', f'{acc:.4f}']
          )
      self.get_logger().info(f'CSVデータを保存しました: {self.csv_filename}')
    except Exception as e:
      self.get_logger().error(f'CSV保存エラー: {e}')

  def plot_graph(self):
    fig, ax1 = plt.subplots(figsize=(9, 5))

    # 速度プロット（左Y軸）
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Velocity [m/s]', color='blue')
    line1 = ax1.plot(
        self.time_log,
        self.target_v_log,
        label='Target Velocity',
        linestyle='--',
        color='blue',
    )
    line2 = ax1.plot(
        self.time_log,
        self.measured_v_log,
        label='Measured Velocity',
        color='red',
    )
    ax1.tick_params(axis='y', labelcolor='blue')
    ax1.grid(True)

    # 加速度プロット（右Y軸：S字軌跡の確認用）
    ax2 = ax1.twinx()
    ax2.set_ylabel('Acceleration [m/s^2]', color='green')
    line3 = ax2.plot(
        self.time_log,
        self.accel_log,
        label='Current Accel',
        linestyle=':',
        color='green',
    )
    ax2.tick_params(axis='y', labelcolor='green')

    # 凡例の統合
    lines = line1 + line2 + line3
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='upper right')

    plt.title('S-Curve (Jerk Controlled) Velocity Profile')
    plt.savefig(self.png_filename)
    self.get_logger().info(f'グラフ画像を保存しました: {self.png_filename}')
    plt.show()

  def emergency_stop(self):
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    self.cmd_pub.publish(twist)


def main(args=None):
  rclpy.init(args=args)
  node = KobukiJerkControlNode()
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