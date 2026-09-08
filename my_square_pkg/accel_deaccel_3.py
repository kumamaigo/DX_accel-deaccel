import csv
import os
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class KobukiAccelDecelTestNode(Node):

  def __init__(self):
    super().__init__('kobuki_accel_decel_test_node')

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
    # 物理パラメータ・摩擦・安全率の修正
    # ----------------------------------------------------
    self.g = 9.81   # 重力加速度 [m/s^2]
    self.mu = 0.25  # 路面（フローリング）の摩擦係数 (0.6 -> 0.25 に変更)
    self.h = 0.19   # 重心の高さ [m]
    self.Lf = 0.15  # 前軸から重心までの距離 [m]
    self.Lr = 0.70  # 後軸から重心までの距離 [m]
    self.L = self.Lf + self.Lr  # ホイールベース（前後の軸間距離） [m]

    # --------- 安全係数周り ----------------
    self.beta = 0.5 # 加速時の安全率 (理論限界値のn%程度に抑える)
    self.alpha = 0.7  # 減速時の安全・調整係数

    self.max_v_kmh = 2.5  # 最高速度の設定 (時速2.5km -> 約0.694 m/s)
    self.v_max = self.max_v_kmh / 3.6  # [m/s]に変換

    # 加速限界・減速限界の計算メソッドを呼び出して設定
    self.a_acc = self.calc_accel_limit()  # 加速限界
    self.a_dec = self.calc_decel_limit()  # 減速限界

    self.get_logger().info(
        f'【計算値】加速限界 (安全率適用後): {self.a_acc:.3f} m/s^2, 減速限界:'
        f' {self.a_dec:.3f} m/s^2'
    )

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
    self.last_control_time = None  # 実時間差 (actual_dt) 計算用の時刻記録
    self.time_log = []
    self.state_log = []
    self.target_v_log = []
    self.measured_v_log = []
    self.data_saved = False

    self.csv_filename = 'accel_deaccel_result.csv'
    self.png_filename = 'accel_deaccel_result.png'

  def calc_accel_limit(self):
    """物理パラメータおよび安全率に基づいて加速限界 [m/s^2] を計算"""
    a_acc_theoretical = (self.mu * self.g * self.Lr) / (
        self.L + self.mu * self.h
    )
    # 理論値に安全率 beta を乗算して急発進とスリップを防止
    return float(a_acc_theoretical * self.beta)

  def calc_decel_limit(self):
    """物理パラメータに基づいて減速限界 [m/s^2] を計算"""
    a_dec = (
        (self.mu * self.g * self.Lf) / (self.L - self.mu * self.h)
    ) * self.alpha
    return float(a_dec)

# ----- オドメトリからスピードを読み込む部分 -----
  def odom_callback(self, msg):
    self.current_v_odom = msg.twist.twist.linear.x

# ----- 実際の経過時間を正確に測る処理 --------
  def control_loop(self):
    now = self.get_clock().now()

    # 初回実行時の時刻初期化
    if self.start_time is None:
      self.start_time = now
      self.last_control_time = now
      return

    # 前回の実行からの【実際の経過時間 (actual_dt)】を計算 [秒]
    actual_dt = (now - self.last_control_time).nanoseconds / 1e9
    self.last_control_time = now

    # 時間計算の異常値ガード (極端なゼロや負数の発生を予防)
    if actual_dt <= 0.0:
      actual_dt = self.dt

    # 制御開始からの総経過時間の計算（秒単位）
    elapsed_total = (now - self.start_time).nanoseconds / 1e9

    # --- 状態遷移 ---
    if self.state == 'ACCEL':
      # 固定値 self.dt ではなく actual_dt を使用してタイマーのブレを自動補正
      self.target_v += self.a_acc * actual_dt
      if self.target_v >= self.v_max:
        self.target_v = self.v_max
        self.state = 'CRUISE'
        self.cruise_start_time = now

    elif self.state == 'CRUISE':
      elapsed_cruise = (now - self.cruise_start_time).nanoseconds / 1e9
      if elapsed_cruise >= 3.0:  # 3秒間定速走行
        self.state = 'DECEL'

    elif self.state == 'DECEL':
      # 減速時も actual_dt を使用
      self.target_v -= self.a_dec * actual_dt
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
        f'[{self.state}] 時間: {elapsed_total:.1f}s | 目標:'
        f' {self.target_v:.2f} m/s | 実測: {self.current_v_odom:.2f} m/s'
    )

    # 送信
    twist = Twist()
    twist.linear.x = float(self.target_v)
    twist.angular.z = 0.0
    self.cmd_pub.publish(twist)

# -------------------------------------------------
# --------------    結果出力    --------------------
# -------------------------------------------------
  def save_data_and_plot(self):
    """CSV出力およびグラフプロットの実行"""
    self.export_to_csv()
    self.plot_graph()

  def export_to_csv(self):
    """実験データをCSV形式で出力"""
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
        ])
        for t, st, v_tgt, v_meas in zip(
            self.time_log, self.state_log, self.target_v_log, self.measured_v_log
        ):
          writer.writerow([f'{t:.3f}', st, f'{v_tgt:.4f}', f'{v_meas:.4f}'])
      self.get_logger().info(
          f'【CSV保存完了】データファイルを {self.csv_filename} に保存しました。'
      )
    except Exception as e:
      self.get_logger().error(f'CSV保存中にエラーが発生しました: {e}')

  def plot_graph(self):
    """実験データのプロットと保存"""
    plt.figure(figsize=(9, 5))
    plt.plot(
        self.time_log,
        self.target_v_log,
        label='Target Velocity (m/s)',
        linestyle='--',
        color='blue',
    )
    plt.plot(
        self.time_log,
        self.measured_v_log,
        label='Measured Velocity (Odom)',
        color='red',
    )

    plt.title('Kobuki Velocity Response (Accel / Cruise / Decel)')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.grid(True)
    plt.legend()

    plt.savefig(self.png_filename)
    self.get_logger().info(
        f'【グラフ保存完了】{self.png_filename} に画像を保存しました。'
    )
    plt.show()

# -----------------------------------------------------------------

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