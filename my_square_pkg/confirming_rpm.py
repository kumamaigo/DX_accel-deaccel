import csv
from can_msgs.msg import Frame
import math
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

#デバック付きのやつ

class WheelSlipDetectorNode(Node):

  def __init__(self):
    super().__init__('wheel_slip_detector')

    # --- 判定パラメータ ---
    self.declare_parameter('rpm_accel_threshold', 1000.0)  # 閾値 [RPM/s]
    self.declare_parameter('lpf_alpha', 0.2)  # LPF平滑化係数
    self.declare_parameter('consecutive_count_thresh', 3)  # 連続判定回数
    self.declare_parameter('wheel_radius', 0.05)  # 車輪半径 [m]

    self.threshold = self.get_parameter('rpm_accel_threshold').value
    self.alpha = self.get_parameter('lpf_alpha').value
    self.consecutive_thresh = self.get_parameter(
        'consecutive_count_thresh'
    ).value
    self.r = self.get_parameter('wheel_radius').value

    # --- 状態変数 ---
    self.prev_time = None
    self.prev_rpm_left = 0
    self.prev_rpm_right = 0

    self.filtered_accel_left = 0.0
    self.filtered_accel_right = 0.0

    self.counter_left = 0
    self.counter_right = 0

    # --- データログ保存用配列 ---
    self.start_time = None
    self.time_log = []
    self.rpm_left_log = []
    self.rpm_right_log = []
    self.v_robot_log = []  # 機体速度保存用
    self.accel_left_log = []
    self.accel_right_log = []
    self.slip_log = []
    self.graph_saved = False

    # --- 通信処理 ---
    self.can_sub = self.create_subscription(
        Frame, 'sub_can', self.can_callback, 10
    )
    self.slip_pub = self.create_publisher(Bool, 'wheel_slip_detected', 10)

  def can_callback(self, msg: Frame):
    if msg.id != 0x211:
      return

    now = self.get_clock().now()
    if self.start_time is None:
      self.start_time = now

    elapsed_total = (now - self.start_time).nanoseconds / 1e9

    # CANデータから左右RPMをパース
    rpm_left = int.from_bytes(msg.data[0:4], byteorder='little', signed=True)
    rpm_right = int.from_bytes(msg.data[4:8], byteorder='little', signed=True)

    # 初回受信時の初期化
    if self.prev_time is None:
      self.prev_time = now
      self.prev_rpm_left = rpm_left
      self.prev_rpm_right = rpm_right
      return

    # 時間差分 dt [s] 計算
    dt = (now - self.prev_time).nanoseconds / 1e9
    if dt <= 0.0:
      return

    # 1. 時間微分 (車輪加速度 [RPM/s])
    raw_accel_left = (rpm_left - self.prev_rpm_left) / dt
    raw_accel_right = (rpm_right - self.prev_rpm_right) / dt

    # 2. 一次ローパスフィルタ (LPF)
    self.filtered_accel_left = (
        1.0 - self.alpha
    ) * self.filtered_accel_left + self.alpha * raw_accel_left
    self.filtered_accel_right = (
        1.0 - self.alpha
    ) * self.filtered_accel_right + self.alpha * raw_accel_right

    # 3. 判定
    slip_left = self._evaluate_slip(self.filtered_accel_left, 'left')
    slip_right = self._evaluate_slip(self.filtered_accel_right, 'right')
    is_slipping = slip_left or slip_right

    # 4. パブリッシュ
    slip_msg = Bool()
    slip_msg.data = is_slipping
    self.slip_pub.publish(slip_msg)

    # 5. RPMから機体速度 [m/s] を計算
    v_left = (rpm_left * 2.0 * math.pi / 60.0) * self.r
    v_right = (rpm_right * 2.0 * math.pi / 60.0) * self.r
    v_robot = (v_left + v_right) / 2.0  # 左右平均速度

    # 6. メモリへログ蓄積
    self.time_log.append(elapsed_total)
    self.rpm_left_log.append(rpm_left)
    self.rpm_right_log.append(rpm_right)
    self.v_robot_log.append(v_robot)
    self.accel_left_log.append(self.filtered_accel_left)
    self.accel_right_log.append(self.filtered_accel_right)
    self.slip_log.append(1 if is_slipping else 0)

    if is_slipping:
      self.get_logger().warn(
          f'【空転検知】左: {self.filtered_accel_left:.1f} RPM/s | 右:'
          f' {self.filtered_accel_right:.1f} RPM/s'
      )

    # 状態更新
    self.prev_time = now
    self.prev_rpm_left = rpm_left
    self.prev_rpm_right = rpm_right

  def _evaluate_slip(self, accel: float, side: str) -> bool:
    if abs(accel) > self.threshold:
      if side == 'left':
        self.counter_left += 1
        return self.counter_left >= self.consecutive_thresh
      else:
        self.counter_right += 1
        return self.counter_right >= self.consecutive_thresh
    else:
      if side == 'left':
        self.counter_left = 0
      else:
        self.counter_right = 0
      return False

  def save_csv_and_plot(self):
    if self.graph_saved or len(self.time_log) == 0:
      return

    # --- 1. CSV保存 ---
    csv_filename = 'rpm_slip_result.csv'
    try:
      with open(csv_filename, mode='w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Time[s]',
            'RPM_Left',
            'RPM_Right',
            'Velocity_Robot[m/s]',
            'Accel_Left[RPM/s]',
            'Accel_Right[RPM/s]',
            'Is_Slipping',
        ])
        for t, rl, rr, v_bot, al, ar, slip in zip(
            self.time_log,
            self.rpm_left_log,
            self.rpm_right_log,
            self.v_robot_log,
            self.accel_left_log,
            self.accel_right_log,
            self.slip_log,
        ):
          writer.writerow(
              [f'{t:.3f}', rl, rr, f'{v_bot:.3f}', f'{al:.2f}', f'{ar:.2f}', slip]
          )
      self.get_logger().info(
          f'【CSV保存完了】{csv_filename} に保存しました。'
      )
    except Exception as e:
      self.get_logger().error(f'CSV保存失敗: {e}')

    # --- 2. グラフ描画・保存 (2段構造 ＋ 右軸に速度表示) ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    # 上段: RPM変動 ＋ 機体速度
    ax1.plot(self.time_log, self.rpm_left_log, label='RPM Left', color='blue')
    ax1.plot(self.time_log, self.rpm_right_log, label='RPM Right', color='cyan')
    ax1.set_ylabel('Wheel Speed [RPM]', color='blue')
    ax1.set_title('Wheel RPM / Speed & Calculated Acceleration')
    ax1.grid(True)

    # 上段の第2Y軸（速度表示）
    ax1_sub = ax1.twinx()
    ax1_sub.plot(
        self.time_log,
        self.v_robot_log,
        label='Robot Speed [m/s]',
        color='green',
        linestyle=':',
    )
    ax1_sub.set_ylabel('Velocity [m/s]', color='green')

    # 凡例の結合
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax1_sub.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')

    # 下段: 車輪加速度と閾値
    ax2.plot(
        self.time_log,
        self.accel_left_log,
        label='Accel Left [RPM/s]',
        color='red',
    )
    ax2.plot(
        self.time_log,
        self.accel_right_log,
        label='Accel Right [RPM/s]',
        color='orange',
    )
    ax2.axhline(
        y=self.threshold,
        color='black',
        linestyle='--',
        label=f'Threshold (+{self.threshold})',
    )
    ax2.axhline(
        y=-self.threshold,
        color='black',
        linestyle='--',
        label=f'Threshold (-{self.threshold})',
    )
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Acceleration [RPM/s]')
    ax2.grid(True)
    ax2.legend(loc='upper left')

    plt.tight_layout()
    png_filename = 'rpm_slip_result.png'
    plt.savefig(png_filename)
    self.get_logger().info(
        f'【グラフ保存完了】{png_filename} に保存しました。'
    )
    self.graph_saved = True

    try:
      plt.show()
    except Exception:
      pass


def main(args=None):
  rclpy.init(args=args)
  node = WheelSlipDetectorNode()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.save_csv_and_plot()
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()