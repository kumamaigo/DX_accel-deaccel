import rclpy  # ROS 2の基本Pythonライブラリ
from rclpy.node import Node  # ノード（処理の最小単位）を作成するためのクラス
from geometry_msgs.msg import Twist  # ロボットへ送る速度命令（直進速度・回転速度）のデータ型
from nav_msgs.msg import Odometry  # ロボットから届くオドメトリ（現在位置・現在速度）のデータ型
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy  # 通信品質（QoS）設定用のクラス

class KobukiAccelDecelTestNode(Node):
    def __init__(self):
        # ノード名を 'kobuki_accel_decel_test_node' として初期化
        super().__init__('kobuki_accel_decel_test_node')

        # ----------------------------------------------------
        # 1. ロボットへの命令出力（Publisher）の設定
        # ----------------------------------------------------
        # Kobukiの速度制御用トピック '/commands/velocity' へTwist型のメッセージを送る準備
        self.cmd_pub = self.create_publisher(Twist, '/commands/velocity', 10)

        # ----------------------------------------------------
        # 2. ロボットからの情報受信（Subscriber）の設定
        # ----------------------------------------------------
        # 前回のコードで動作確認が取れた通信品質（QoS）をそのまま使用
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # データを確実に届ける設定
            durability=DurabilityPolicy.VOLATILE,    # 過去のデータは保持しない設定
            depth=10                                 # 保持するデータ数
        )
        # '/odom' トピックからオドメトリデータを受け取り、受信のたびに odom_callback 関数を実行
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )

        # ----------------------------------------------------
        # 3. 物理パラメータの設定（実験条件に合わせて変更してください）
        # ----------------------------------------------------
        self.g = 9.81         # 重力加速度 [m/s^2]
        self.mu = 0.6         # 路面の摩擦係数
        self.h = 0.15         # 重心の高さ [m]
        self.Lf = 0.12        # 前軸から重心までの距離 [m]
        self.Lr = 0.12        # 後軸から重心までの距離 [m]
        self.L = self.Lf + self.Lr  # ホイールベース（前後の軸間距離） [m]
        self.alpha = 1.0      # 減速時の安全・調整係数
        
        # 最高速度の設定 (時速6km = 約1.67m/s)
        # 最初は低速で行う（1回目は1.0）
        self.max_v_kmh = 4.0
        self.v_max = self.max_v_kmh / 3.6  # [m/s] 単位に換算

        """----------------------------------------------------
        # 4. 手書きノートの式から限界加減速度を自動計算
        # ----------------------------------------------------
        #self.a_acc = self.calc_accel_limit()  # 加速限界
        #self.a_dec = self.calc_decel_limit()  # 減速限界
        """
        # （要変更） Kobukiに向けて安定した速度を仮入力
        self.a_acc = 0.8  # 加速限界
        self.a_dec = 0.8  # 減速限界

        # 計算された加減速度をターミナル画面（ログ）に表示
        self.get_logger().info(f"【計算限界】最大加速度: {self.a_acc:.2f} m/s^2 | 最大減速度: {self.a_dec:.2f} m/s^2")

        

        # ----------------------------------------------------
        # 5. 制御ループ用の変数設定
        # --------------a--------------------------------------
        self.current_v_odom = 0.0   # オドメトリから読み取った実測速度 [m/s]
        self.target_v = 0.0         # ロボットに指示する目標速度 [m/s]
        self.state = 'ACCEL'        # 状態管理（'ACCEL':加速, 'CRUISE':定速, 'DECEL':減速, 'DONE':終了）
        self.cruise_start_time = None  # 定速走行を開始した時刻を記録する変数

        # 0.1秒ごと（10Hz）に control_loop 関数を繰り返し呼ぶタイマーを起動
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)

    def calc_accel_limit(self):
        """ 手書きノートの式に基づく「最大加速度」の計算 """
        term1 = (self.mu * self.g * self.Lr) / (self.L + self.mu * self.h)  # 駆動輪のスリップ限界
        term2 = self.g * (self.Lr / self.h)                                 # ウイリー（前輪浮き）限界
        term3 = self.mu * self.g                                            # タイヤの摩擦限界（直進のため w=0）
        # 3つの限界条件の中で一番小さい値（最も厳しい制限）を採用する
        return min(term1, term2, term3)

    def calc_decel_limit(self):
        """ 手書きノートの式に基づく「最大減速度」の計算 """
        term1 = (self.mu * self.g * self.Lr) / (self.L - self.mu * self.h)  # 駆動輪のスリップ限界
        term2 = self.g * (self.Lf / self.h)                                 # 前転（ジャックナイフ）限界
        term3 = self.mu * self.g                                            # タイヤの摩擦限界（直進のため w=0）
        # 3つの限界条件の最小値に調整係数 alpha を掛ける
        return self.alpha * min(term1, term2, term3)

    def odom_callback(self, msg):
        """ オドメトリデータが届くたびに更新される関数 """
        # ロボットが実際に現在出している前後方向の速度 [m/s] を取得
        self.current_v_odom = msg.twist.twist.linear.x

    def control_loop(self):
        """ 0.1秒ごとに実行されるメインの制御関数 """
        twist = Twist()  # ロボットに送る速度コマンドの入れ物を作成

        # --- 【状態 1: 加速フェーズ】 ---
        if self.state == 'ACCEL':
            # 目標速度を「加速度 × 時間(0.1秒)」分だけ増やす
            self.target_v += self.a_acc * self.dt
            self.get_logger().info(f"[加速中] 目標速度: {self.target_v:.2f} m/s | オドメトリ計測: {self.current_v_odom:.2f} m/s")
            
            # 最高速度に達したら定速走行へ移行
            if self.target_v >= self.v_max:
                self.target_v = self.v_max
                self.state = 'CRUISE'
                self.cruise_start_time = self.get_clock().now()  # 現在時刻を記録

        # --- 【状態 2: 定速走行フェーズ（1秒間）】 ---
        elif self.state == 'CRUISE':
            self.get_logger().info(f"[定速走行中] 目標速度: {self.target_v:.2f} m/s | オドメトリ計測: {self.current_v_odom:.2f} m/s")
            # 定速走行を開始してからの経過時間を計算（秒単位）
            elapsed = (self.get_clock().now() - self.cruise_start_time).nanoseconds / 1e9
            
            # 1秒経過したら減速フェーズへ移行
            if elapsed >= 3.0:
                self.state = 'DECEL'

        # --- 【状態 3: 減速フェーズ】 ---
        elif self.state == 'DECEL':
            # 目標速度を「減速度 × 時間(0.1秒)」分だけ減らす
            self.target_v -= self.a_dec * self.dt
            self.get_logger().info(f"[減速中] 目標速度: {self.target_v:.2f} m/s | オドメトリ計測: {self.current_v_odom:.2f} m/s")
            
            # 速度が0以下になったら終了へ移行
            if self.target_v <= 0.0:
                self.target_v = 0.0
                self.state = 'DONE'

        # --- 【状態 4: 終了（安全停止）】 ---
        elif self.state == 'DONE':
            self.get_logger().info("[完了] 実験終了・安全停止中")
            self.target_v = 0.0

        # 作成した目標速度をデータにセットしてパブリッシュ（送信）
        twist.linear.x = float(self.target_v)
        twist.angular.z = 0.0  # 直進のため回転速度は 0
        self.cmd_pub.publish(twist)

    def emergency_stop(self):
        """ Ctrl+C などの割り込み時にロボットへ即座に速度0を送る関数 """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().warn("【緊急停止】速度 0 を送信しました。")

def main(args=None):
    rclpy.init(args=args)  # ROS 2のPython通信システムを初期化
    node = KobukiAccelDecelTestNode()  # ノードのインスタンスを生成
    
    try:
        rclpy.spin(node)  # ノードを実行（タイマーやイベントを待ち受ける）
    except KeyboardInterrupt:
        # キーボードで Ctrl + C が押された場合に緊急停止を実行
        # node.emergency_stop()
        pass
    
    finally:
        # プログラム終了時に確実に停止命令を送り、処理を安全に閉じる
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()