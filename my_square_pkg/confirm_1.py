import csv
from can_msgs.msg import Frame
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node


class HighPrecisionRpmObserverNode(Node):

  def __init__(self):
    super().__init__('high_precision_rpm_observer')

    # ログ用配列
    self.start_time = None
    self.time_log = []
    self.rpm_left_log = []
    self.rpm_right_log = []

    # サブスクライバ設定
    self.can_sub = self.create_subscription(
        Frame, 'sub_can', self.can_callback, 100
    )
    self.get_logger().info(
        '【高精度RPM観測ノード起動】CANデータを記録中... (終了時は Ctrl+C)'
    )

  def can_callback(self, msg: Frame):
    if msg.id != 0x211:
      return

    now = self.get_clock().now()
    if self.start_time is None:
      self.start_time = now

    elapsed_sec = (now - self.start_time).nanoseconds / 1e9

    # CANデータのパース (32bit signed, Little Endian)
    rpm_left = int.from_bytes(msg.data[0:4], byteorder='little', signed=True)
    rpm_right = int.from_bytes(msg.data[4:8], byteorder='little', signed=True)

    # 配列への保存（最高速処理）
    self.time_log.append(elapsed_sec)
    self.rpm_left_log.append(rpm_left)
    self.rpm_right_log.append(rpm_right)

  def save_data(self):
    if len(self.time_log) == 0:
      self.get_logger().warn('データが記録されませんでした。')
      return

    # CSV出力
    csv_file = 'precision_rpm_log.csv'
    with open(csv_file, 'w', newline='', encoding='utf-8') as f:
      writer = csv.writer(f)
      writer.writerow(['Time[s]', 'RPM_Left', 'RPM_Right'])
      for t, l, r in zip(
          self.time_log, self.rpm_left_log, self.rpm_right_log
      ):
        writer.writerow([f'{t:.4f}', l, r])
    self.get_logger().info(f'【CSV保存完了】{csv_file}')

    # グラフ画像保存
    plt.figure(figsize=(10, 5))
    plt.plot(self.time_log, self.rpm_left_log, label='RPM Left', color='blue')
    plt.plot(self.time_log, self.rpm_right_log, label='RPM Right', color='red')
    plt.xlabel('Time [s]')
    plt.ylabel('Speed [RPM]')
    plt.title('High-Precision Wheel RPM Log')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    png_file = 'precision_rpm_graph.png'
    plt.savefig(png_file)
    self.get_logger().info(f'【グラフ保存完了】{png_file}')


def main(args=None):
  rclpy.init(args=args)
  node = HighPrecisionRpmObserverNode()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.save_data()
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()