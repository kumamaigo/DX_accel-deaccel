import rclpy  # ROS 2の基本Pythonライブラリ
from rclpy.node import Node  # ノード（処理の最小単位）を作成するためのクラス
from geometry_msgs.msg import Twist  # ロボットへ送る速度命令（直進速度・回転速度）のデータ型
from nav_msgs.msg import Odometry  # ロボットから届くオドメトリ（現在位置・現在速度）のデータ型
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy  # 通信品質（QoS）設定用のクラス
import matplotlib.pyplot as plt  # グラフ描画用

class KobukiAccelDecelTestNode(Node):
    def __init__(self):
        # ノード名を 'kobuki_accel_decel_test_node' として初期化
        super().__init__('kobuki_accel_decel_test_node')

        # ----------------------------------------------------
        # 1. ロボットへの命令出力（Publisher）の設定
        # ----------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, '/aiformula_control/twist_mux/cmd_vel', 10)

        # ----------------------------------------------------
        # 2. ロボットからの情報受信（Subscriber）の設定
        # ----------------------------------------------------
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # データを確実に届ける設定
            durability=DurabilityPolicy.VOLATILE,    # 過去のデータは保持しない設定
            depth=10                                 # 保持するデータ数
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/aiformula_sensing/gyro_odometry_publisher/odom',
            self.odom_callback,
            qos_profile
        )

        # ----------------------------------------------------
        # 3. 物理パラメータの設定
        # ----------------------------------------------------
        self.g = 9.81         # 重力加速度 [m/s^2]
        self.mu = 0.6         # 路面の摩擦係数
        self.h = 0.15         # 重心の高さ [m]
        self.Lf = 0.12        # 前軸から重心までの距離 [m]
        self.Lr = 0.12        # 後軸から重心までの距離 [m]
        self.L = self.Lf + self.Lr  # ホイールベース [m]
        self.alpha = 1.0      # 減速時の安全・調整係数
        
        # 最高速度の設定 (時速4km = 約1.11m/s)
        self.max_v_kmh = 4.0
        self.v_max = self.max_v_kmh / 3.6  # [m/s] 単位に換算

        # ----------------------------------------------------
        # 4. 手書きノートの式から限界加減速度を自動計算（有効化）
        # ----------------------------------------------------
        self.a_acc = self.calc_accel_limit()  # 加速限界
        self.a_dec = self.calc_decel_limit()  # 減速限界

        # 計算された加減速度をターミナル画面（ログ）に表示
        self.get_logger().info(f"【計算限界】最大加速度: {self.a_acc:.2f} m/s^2 | 最大減速度: {self.a_dec:.2f} m/s^2")

        # ----------------------------------------------------
        # 5. 制御ループ用の変数設定
        # ----------------------------------------------------
        self.current_v_odom = 0.0   # オドメトリから読み取った実測速度 [m/s]
        self.target_v = 0.0         # ロボットに指示する目標速度 [m/s]
        self.state = 'ACCEL'        # 状態管理（'ACCEL', 'CRUISE', 'DECEL', 'DONE'）
        self.cruise_start_time = None

        # 0.1秒ごと（10Hz）に control_loop 関数を繰り返し呼ぶタイマーを起動
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)

        # ----------------------------------------------------
        # 6. グラフ保存用のデータリスト＆時間管理
        # ----------------------------------------------------
        self.start_time = None
        self.time_log = []
        self.target_v_log = []
        self.measured_v_log = []
        self.graph_saved = False

    def calc_accel_limit(self):
        """ 手書きノートの式に基づく「最大加速度」の計算 """
        term1 = (self.mu * self.g * self.Lr) / (self.L + self.mu * self.h)  # 駆動輪のスリップ限界
        term2 = self.g * (self.Lr / self.h)                                 # ウイリー限界
        term3 = self.mu * self.g                                            # タイヤの摩擦限界
        return min(term1, term2, term3)

    def calc_decel_limit(self):
        """ 手書きノートの式に基づく「最大減速度」の計算 """
        term1 = (self.mu * self.g * self.Lr) / (self.L - self.mu * self.h)  # 駆動輪のスリップ限界
        term2 = self.g * (self.Lf / self.h)                                 # 前転限界
        term3 = self.mu * self.g                                            # タイヤの摩擦限界
        return self.alpha * min(term1, term2, term3)

    def odom_callback(self, msg):
        """ オドメトリデータが届くたびに更新される関数 """
        self.current_v_odom = msg.twist.twist.linear.x

    def control_loop(self):
        """ 0.1秒ごとに実行されるメインの制御関数 """
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        
        elapsed_total = (now - self.start_time).nanoseconds / 1e9

        # --- 【状態 1: 加速フェーズ】 ---
        if self.state == 'ACCEL':
            self.target_v += self.a_acc * self.dt
            if self.target_v >= self.v_max:
                self.target_v = self.v_max
                self.state = 'CRUISE'
                self.cruise_start_time = now

        # --- 【状態 2: 定速走行フェーズ（3秒間）】 ---
        elif self.state == 'CRUISE':
            elapsed_cruise = (now - self.cruise_start_time).nanoseconds / 1e9
            if elapsed_cruise >= 3.0:
                self.state = 'DECEL'

        # --- 【状態 3: 減速フェーズ】 ---
        elif self.state == 'DECEL':
            self.target_v -= self.a_dec * self.dt
            if self.target_v <= 0.0:
                self.target_v = 0.0
                self.state = 'DONE'

        # --- 【状態 4: 終了（グラフ保存）】 ---
        elif self.state == 'DONE':
            self.target_v = 0.0
            if not self.graph_saved:
                self.save_and_plot_graph()
                self.graph_saved = True

        # データログの蓄積
        self.time_log.append(elapsed_total)
        self.target_v_log.append(self.target_v)
        self.measured_v_log.append(self.current_v_odom)

        self.get_logger().info(
            f"[{self.state}] 時間: {elapsed_total:.1f}s | 目標: {self.target_v:.2f} m/s | 実測: {self.current_v_odom:.2f} m/s"
        )

        # 送信
        twist = Twist()
        twist.linear.x = float(self.target_v)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def save_and_plot_graph(self):
        """ 実験データのプロットと保存 """
        plt.figure(figsize=(9, 5))
        plt.plot(self.time_log, self.target_v_log, label='Target Velocity (m/s)', linestyle='--', color='blue')
        plt.plot(self.time_log, self.measured_v_log, label='Measured Velocity (Odom)', color='red')
        
        plt.title('Kobuki Kinematic Accel/Decel Response')
        plt.xlabel('Time [s]')
        plt.ylabel('Velocity [m/s]')
        plt.grid(True)
        plt.legend()
        
        filename = 'kinematic_accel_deaccel_result.png'
        plt.savefig(filename)
        self.get_logger().info(f"【グラフ保存完了】{filename} に画像を保存しました。")
        plt.show()

    def emergency_stop(self):
        """ Ctrl+C などの割り込み時にロボットへ即座に速度0を送る関数 """
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

if __name__ == '__main__':
    main()