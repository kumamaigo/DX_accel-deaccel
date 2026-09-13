from collections import deque
import threading
import time
from can_msgs.msg import Frame
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node


class LiveRpmObserverNode(Node):

  def __init__(self, max_points=200):
    super().__init__('live_rpm_observer')

    self.start_time = None
    # 直近N点だけを保持するキュー（描画負荷軽減）
    self.time_queue = deque(maxlen=max_points)
    self.rpm_left_queue = deque(maxlen=max_points)
    self.rpm_right_queue = deque(maxlen=max_points)

    self.can_sub = self.create_subscription(
        Frame, 'sub_can', self.can_callback, 10
    )

  def can_callback(self, msg: Frame):
    if msg.id != 0x211:
      return

    now = self.get_clock().now()
    if self.start_time is None:
      self.start_time = now

    elapsed_sec = (now - self.start_time).nanoseconds / 1e9

    rpm_left = int.from_bytes(msg.data[0:4], byteorder='little', signed=True)
    rpm_right = int.from_bytes(msg.data[4:8], byteorder='little', signed=True)

    self.time_queue.append(elapsed_sec)
    self.rpm_left_queue.append(rpm_left)
    self.rpm_right_queue.append(rpm_right)


def main(args=None):
  rclpy.init(args=args)
  node = LiveRpmObserverNode(max_points=200)

  # ROS 2の通信処理をバックグラウンドスレッドで実行
  spin_thread = threading.Thread(
      target=rclpy.spin, args=(node,), daemon=True
  )
  spin_thread.start()

  # メインスレッドでリアルタイムグラフの描画処理
  plt.ion()  # インタラクティブモードON
  fig, ax = plt.subplots(figsize=(8, 4))
  (line_left,) = ax.plot([], [], label='RPM Left', color='blue')
  (line_right,) = ax.plot([], [], label='RPM Right', color='red')

  ax.set_xlabel('Time [s]')
  ax.set_ylabel('Speed [RPM]')
  ax.set_title('Real-time RPM Monitor')
  ax.grid(True)
  ax.legend(loc='upper left')

  try:
    while rclpy.ok():
      if len(node.time_queue) > 0:
        # データの更新
        times = list(node.time_queue)
        rpm_l = list(node.rpm_left_queue)
        rpm_r = list(node.rpm_right_queue)

        line_left.set_data(times, rpm_l)
        line_right.set_data(times, rpm_r)

        # 軸範囲の自動調整
        ax.set_xlim(times[0], times[-1] + 0.1)
        all_rpm = rpm_l + rpm_r
        if len(all_rpm) > 0:
          min_val, max_val = min(all_rpm), max(all_rpm)
          ax.set_ylim(min_val - 50, max_val + 50)

        fig.canvas.draw()
        fig.canvas.flush_events()

      time.sleep(0.05)  # 20Hz更新（描画負荷の調整用）

  except KeyboardInterrupt:
    pass
  finally:
    plt.close('all')
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()