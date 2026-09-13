import csv
import math
import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class KobukiSlipDetectionTestNode(Node):

  def __init__(self):
    # ノード名を「kobuki_slip_detection_node」として初期化
    super().__init__('kobuki_slip_detection_node')

    # =========================================================================
    #  ⚙️ 1. 空転（スリップ）検知機能のオン/オフ ＆ 閾値設定
    # =========================================================================
    self.enable_slip_detection = True  # True: 空転判定を行う / False: 判定を行わない（計測のみ）
    self.slip_threshold = 0.15  # 空転とみなす速度差 [m/s] (駆動輪速度 - 従属輪速度 > 0.15m/s で検出)

    # =========================================================================
    #  📐 2. 機体・路面の物理パラメータ設定（実験条件）
    # =========================================================================
    self.g = 9.81  # 重力加速度 [m/s^2]
    self.mu = 0.49  # 路面の静止摩擦係数
    self.h = 0.15  # 重心高さ [m]
    self.Lf = 0.173  # 前軸から重心までの距離 [m]
    self.Lr = 0.542  # 後軸から重心までの距離 [m]
    self.L = self.Lf + self.Lr  # ホイールベース（前軸〜後軸の全幅） [m]

    # --- 加減速の安全係数 ---
    self.alpha = 0.8  # 加速限界にかける安全係数 (1.0で理論限界値)
    self.beta = 0.5  # 減速限界にかける安全係数 (1.0で理論限界値)

    # --- 目標速度プロファイルの設定 ---
    self.v_max1 = 3.6 / 3.6  # 第1段階の目標速度: 3.6 km/h = 1.0 m/s
    self.v_max2 = 7.2 / 3.6  # 第2段階の目標速度: 7.2 km/h = 2.0 m/s
    self.cruise1_time = 0.8  # 第1定速フェーズの保持時間 [s]
    self.cruise2_time = 0.8  # 第2定速フェーズの保持時間 [s]

    # =========================================================================
    #  📊 3. 物理理論限界値の計算（摩擦・重心に基づく最大加減速度）
    # =========================================================================
    raw_a_acc = self.calc_accel_limit()  # 摩擦限界による理論最大加速度
    raw_a_dec = self.calc_decel_limit()  # 摩擦限界による理論最大減速度

    # 安全係数を掛け合わせた「実際に使用する加減速度」
    self.a_acc = raw_a_acc * self.alpha
    self.a_dec = raw_a_dec * self.beta

    # 起動ログに設定パラメータを出力
    self.get_logger().info('=== 実験パラメータの設定完了 ===')
    self.get_logger().info(
        f'【スリップ設定】 検知有効: {self.enable_slip_detection} | 閾値:'
        f' {self.slip_threshold:.2f} m/s'
    )
    self.get_logger().info(
        f'【物理パラメータ】 mu={self.mu}, h={self.h}m, Lf={self.Lf}m,'
        f' Lr={self.Lr}m, L={self.L}m'
    )
    self.get_logger().info(
        f'【限界加速度】 理論限界: {raw_a_acc:.2f} m/s² -> 適用値(α={self.alpha}):'
        f' {self.a_acc:.2f} m/s²'
    )
    self.get_logger().info(
        f'【限界減速度】 理論限界: {raw_a_dec:.2f} m/s² -> 適用値(β={self.beta}):'
        f' {self.a_dec:.2f} m/s²'
    )

    # =========================================================================
    #  📡 4. ROS 2 通信の設定 (Publisher / Subscriber)
    # =========================================================================
    # [送信] モータードライバーへの速度指令
    self.cmd_pub = self.create_publisher(
        Twist, '/aiformula_control/twist_mux/cmd_vel', 10
    )

    # [受信設定] ドロップを防ぐ軽量なQoS設定
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        depth=1,
    )

    # [受信1] 駆動輪オドメトリ (モーター回転数から計算された速度)
    self.drive_odom_sub = self.create_subscription(
        Odometry,
        '/aiformula_sensing/gyro_odometry_publisher/odom',
        self.drive_odom_callback,
        qos_profile,
    )

    # [受信2] 従属輪オドメトリ (encoder_driver.cpp から送られてくる実速度)
    self.sub_odom_sub = self.create_subscription(
        Odometry, 'sub_odom', self.sub_odom_callback, 10
    )

    # =========================================================================
    #  🔄 5. 内部変数・制御タイマーの設定
    # =========================================================================
    self.current_v_drive = 0.0  # 駆動輪から計測された現在の速度
    self.current_v_sub = 0.0  # 従属輪から計測された現在の速度
    self.target_v = 0.0  # ロボットに命令する目標速度
    self.state = 'ACCEL1'  # 状態遷移の初期ステート（第1加速）
    self.phase_start_time = None

    self.dt = 0.1  # 制御ループの周期 [s] (0.1秒 = 10Hz)
    self.timer = self.create_timer(self.dt, self.control_loop)

    # --- 記録データ保持用リスト ---
    self.start_time = None
    self.time_log = []  # 経過時間 [s]
    self.target_v_log = []  # 指令速度 [m/s]
    self.drive_v_log = []  # 駆動輪実測速度 [m/s]
    self.sub_v_log = []  # 従属輪実測速度 [m/s]
    self.slip_amount_log = []  # 速度差 (駆動 - 従属) [m/s]
    self.is_slipping_log = []  # 空転フラグ (True / False)
    self.graph_saved = False

  # --- 物理限界加速度の算出式 ---
  def calc_accel_limit(self):
    term1 = (self.mu * self.g * self.Lr) / (self.L + self.mu * self.h)
    term2 = self.g * (self.Lr / self.h)
    term3 = self.mu * self.g
    return min(term1, term2, term3)

  # --- 物理限界減速度の算出式 ---
  def calc_decel_limit(self):
    term1 = (self.mu * self.g * self.Lr) / (self.L - self.mu * self.h)
    term2 = self.g * (self.Lf / self.h)
    term3 = self.mu * self.g
    return min(term1, term2, term3)

  # --- [コールバック] 駆動輪オドメトリ受信処理 ---
  def drive_odom_callback(self, msg):
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    self.current_v_drive = math.hypot(vx, vy)  # X, Y成分から直進速度を算出

  # --- [コールバック] 従属輪オドメトリ(encoder_driver.cpp)受信処理 ---
  def sub_odom_callback(self, msg):
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    self.current_v_sub = math.hypot(vx, vy)  # X, Y成分から直進速度を算出

  # =========================================================================
  #  ⏱️ 6. メイン制御ループ（0.1秒周期で繰り返し実行される）
  # =========================================================================
  def control_loop(self):
    now = self.get_clock().now()
    if self.start_time is None:
      self.start_time = now

    # テスト開始からの経過時間 [秒]
    elapsed_total = (now - self.start_time).nanoseconds / 1e9

    # -------------------------------------------------------------------------
    #  A) 状態遷移（2段階の加減速プロファイルの生成）
    # -------------------------------------------------------------------------
    # [フェーズ1] 第1加速 (0 -> v_max1)
    if self.state == 'ACCEL1':
      self.target_v += self.a_acc * self.dt
      if self.target_v >= self.v_max1:
        self.target_v = self.v_max1
        self.state = 'CRUISE1'
        self.phase_start_time = now

    # [フェーズ2] 第1定速 (v_max1 を一定時間維持)
    elif self.state == 'CRUISE1':
      elapsed_phase = (now - self.phase_start_time).nanoseconds / 1e9
      if elapsed_phase >= self.cruise1_time:
        self.state = 'ACCEL2'

    # [フェーズ3] 第2加速 (v_max1 -> v_max2)
    elif self.state == 'ACCEL2':
      self.target_v += self.a_acc * self.dt
      if self.target_v >= self.v_max2:
        self.target_v = self.v_max2
        self.state = 'CRUISE2'
        self.phase_start_time = now

    # [フェーズ4] 第2定速 (v_max2 を一定時間維持)
    elif self.state == 'CRUISE2':
      elapsed_phase = (now - self.phase_start_time).nanoseconds / 1e9
      if elapsed_phase >= self.cruise2_time:
        self.state = 'DECEL'

    # [フェーズ5] 減速 (v_max2 -> 0)
    elif self.state == 'DECEL':
      self.target_v -= self.a_dec * self.dt
      if self.target_v <= 0.0:
        self.target_v = 0.0

      # 指令値が0かつ、駆動輪の実測スピードも停止判定以下になったら終了
      if self.target_v == 0.0 and self.current_v_drive <= 0.05:
        self.state = 'DONE'

    # [フェーズ6] 試験終了処理
    elif self.state == 'DONE':
      self.target_v = 0.0
      if not self.graph_saved:
        self.save_and_plot_graph()  # グラフ＆CSVの自動保存を実行
        self.graph_saved = True

    # -------------------------------------------------------------------------
    #  B) 空転（スリップ）の計算と判定
    # -------------------------------------------------------------------------
    # 駆動輪速度 - 従属輪速度 = 駆動輪の空転量
    slip_amount = self.current_v_drive - self.current_v_sub
    is_slipping = False

    # スリップ検知機能がONで、試験走行中の場合のみチェック
    if self.enable_slip_detection and self.state != 'DONE':
      if slip_amount > self.slip_threshold:
        is_slipping = True
        self.get_logger().warn(
            f'⚠️【空転発生】 速度差: {slip_amount:.2f} m/s (駆動:'
            f' {self.current_v_drive:.2f}m/s | 従属: {self.current_v_sub:.2f}m/s)'
        )

    # -------------------------------------------------------------------------
    #  C) データの記録 ＆ コンソール出力
    # -------------------------------------------------------------------------
    self.time_log.append(elapsed_total)
    self.target_v_log.append(self.target_v)
    self.drive_v_log.append(self.current_v_drive)
    self.sub_v_log.append(self.current_v_sub)
    self.slip_amount_log.append(slip_amount)
    self.is_slipping_log.append(is_slipping)

    # ターミナルへリアルタイムログを出力
    self.get_logger().info(
        f'[{self.state}] 時間: {elapsed_total:.1f}s | 指令: {self.target_v:.2f}'
        f' | 駆動: {self.current_v_drive:.2f} | 従属: {self.current_v_sub:.2f}'
    )

    # -------------------------------------------------------------------------
    #  D) ロボットへ速度命令パブリッシュ
    # -------------------------------------------------------------------------
    twist = Twist()
    twist.linear.x = float(self.target_v)
    twist.angular.z = 0.0
    self.cmd_pub.publish(twist)

  # =========================================================================
  #  💾 7. 実験データの保存（CSVファイル ＆ PNGグラフ生成）
  # =========================================================================
  def save_and_plot_graph(self):
    # -------------------------------------------------------------------------
    #  1) CSVファイルの保存（ヘッダーに全ての実験パラメータを出力）
    # -------------------------------------------------------------------------
    csv_filename = 'slip_detection_test_result.csv'
    try:
      with open(csv_filename, mode='w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)

        # 実験条件・設定パラメータの出力
        writer.writerow(['# --- Experimental Parameters & Conditions ---'])
        writer.writerow(
            ['# Enable Slip Detection', self.enable_slip_detection]
        )
        writer.writerow(['# Slip Threshold [m/s]', self.slip_threshold])
        writer.writerow(['# Friction Coefficient (mu)', self.mu])
        writer.writerow(['# Gravity (g) [m/s^2]', self.g])
        writer.writerow(['# Center of Gravity Height (h) [m]', self.h])
        writer.writerow(
            ['# Distance CoG to Front Axle (Lf) [m]', self.Lf]
        )
        writer.writerow(['# Distance CoG to Rear Axle (Lr) [m]', self.Lr])
        writer.writerow(['# Wheelbase (L) [m]', self.L])
        writer.writerow(['# Accel Safety Factor (alpha)', self.alpha])
        writer.writerow(['# Decel Safety Factor (beta)', self.beta])
        writer.writerow(
            ['# Target Speed 1 [m/s]', f'{self.v_max1:.2f}']
        )
        writer.writerow(
            ['# Target Speed 2 [m/s]', f'{self.v_max2:.2f}']
        )
        writer.writerow(
            ['# Applied Accel Limit [m/s^2]', f'{self.a_acc:.3f}']
        )
        writer.writerow(
            ['# Applied Decel Limit [m/s^2]', f'{self.a_dec:.3f}']
        )
        writer.writerow([])

        # 列名（ヘッダー）の出力
        writer.writerow([
            'Time[s]',
            'Target_Vel[m/s]',
            'Drive_Odom_Vel[m/s]',
            'Sub_Odom_Vel[m/s]',
            'Slip_Amount[m/s]',
            'Is_Slipping',
        ])

        # データ行の書き込み
        for t, v_cmd, v_drv, v_sub, slip, is_slip in zip(
            self.time_log,
            self.target_v_log,
            self.drive_v_log,
            self.sub_v_log,
            self.slip_amount_log,
            self.is_slipping_log,
        ):
          writer.writerow([
              f'{t:.3f}',
              f'{v_cmd:.3f}',
              f'{v_drv:.3f}',
              f'{v_sub:.3f}',
              f'{slip:.3f}',
              is_slip,
          ])

      self.get_logger().info(
          f'【CSV保存成功】 {csv_filename} にパラメータとデータを保存しました。'
      )
    except Exception as e:
      self.get_logger().error(f'CSV保存失敗: {e}')

    # -------------------------------------------------------------------------
    #  2) PNGグラフ画像の保存（図の中に全パラメータを表示）
    # -------------------------------------------------------------------------
    plt.figure(figsize=(11, 7))

    # 指令値（青破線）
    plt.plot(
        self.time_log,
        self.target_v_log,
        label='Commanded Vel (cmd_vel)',
        linestyle='--',
        color='blue',
        alpha=0.7,
    )
    # 駆動輪実測（赤実線）
    plt.plot(
        self.time_log,
        self.drive_v_log,
        label='Drive Wheel Odom (Drive)',
        color='red',
        linewidth=1.5,
    )
    # 従属輪実測（緑実線）
    plt.plot(
        self.time_log,
        self.sub_v_log,
        label='Follower Wheel Odom (Sub)',
        color='green',
        linewidth=1.5,
    )

    # 空転検知時のハイライトプロット（オレンジ色のドット）
    if self.enable_slip_detection:
      slip_times = [
          t for t, flag in zip(self.time_log, self.is_slipping_log) if flag
      ]
      slip_vals = [
          v
          for v, flag in zip(self.drive_v_log, self.is_slipping_log)
          if flag
      ]
      if slip_times:
        plt.scatter(
            slip_times,
            slip_vals,
            color='orange',
            s=35,
            zorder=5,
            label='Slip Detected',
        )

    # グラフ内に詳細な実験パラメータテキスト領域を配置
    param_text = (
        f'[ Physics & Experimental Params ]\n'
        f'μ: {self.mu} | h: {self.h}m\n'
        f'Lf: {self.Lf}m | Lr: {self.Lr}m | L: {self.L}m\n'
        f'α: {self.alpha} | β: {self.beta}\n'
        f'a_acc: {self.a_acc:.2f} m/s² | a_dec: {self.a_dec:.2f} m/s²\n'
        f'Slip Detection: {self.enable_slip_detection}\n'
        f'Slip Threshold: {self.slip_threshold} m/s'
    )

    plt.gca().text(
        0.97,
        0.05,
        param_text,
        transform=plt.gca().transAxes,
        fontsize=8.5,
        verticalalignment='bottom',
        horizontalalignment='right',
        bbox=dict(
            boxstyle='round,pad=0.5',
            facecolor='white',
            alpha=0.85,
            edgecolor='gray',
        ),
    )

    plt.title('3-Way Velocity Comparison & Slip Detection Test')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.grid(True)
    plt.legend(loc='upper left')

    png_filename = 'slip_detection_test_result.png'
    plt.savefig(png_filename)
    self.get_logger().info(
        f'【グラフ保存成功】 {png_filename} に実験画像を保存しました。'
    )
    plt.show()

  # --- 緊急停止機能 ---
  def emergency_stop(self):
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    self.cmd_pub.publish(twist)
    self.get_logger().warn('【緊急停止】速度 0 を送信しました。')


def main(args=None):
  rclpy.init(args=args)
  node = KobukiSlipDetectionTestNode()

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


if __name__ == '__main__':
  main()


