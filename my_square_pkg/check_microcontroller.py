import sys
import time
import serial

# デフォルトポート設定（引数で指定しない場合）
DEFAULT_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200


def main():
  port_name = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

  print('==============================================')
  print(' ESP32-S3 エンコーダ動作確認ツール')
  print(f' 接続ポート: {port_name} | ボーレート: {BAUD_RATE}')
  print(' 手でエンコーダを回して数値が動くか確認してください。')
  print(' [Ctrl + C] で終了します。')
  print('==============================================\n')

  try:
    ser = serial.Serial(port_name, BAUD_RATE, timeout=1)
    time.sleep(1.5)  # ポート安定化待ち
    ser.reset_input_buffer()

    while True:
      if ser.in_waiting > 0:
        # シリアルデータの読み込みとパース
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if not line:
          continue

        items = line.split(',')

        # マイコンから出力される10個のデータ項目のフォーマットチェック
        if len(items) == 10:
          try:
            c1, c2 = int(items[0]), int(items[1])
            rad1, w1, rps1, rpm1 = map(float, items[2:6])
            rad2, w2, rps2, rpm2 = map(float, items[6:10])

            # コンソール上書き整形表示
            output = (
                f'\r[Enc1] Count:{c1:7d} | Rad:{rad1:7.2f} | Vel:{w1:6.2f}'
                f' rad/s | RPM:{rpm1:6.1f}  ||  [Enc2] Count:{c2:7d} |'
                f' Rad:{rad2:7.2f} | Vel:{w2:6.2f} rad/s | RPM:{rpm2:6.1f}'
            )
            sys.stdout.write(output)
            sys.stdout.flush()
          except ValueError:
            pass

  except serial.SerialException as e:
    print(f'\n[エラー] シリアルポートを開けませんでした: {e}')
    print('※ ポート番号やアクセス権限を確認してください。')
  except KeyboardInterrupt:
    print('\n\n動作確認を終了しました。')
  finally:
    if 'ser' in locals() and ser.is_open:
      ser.close()


if __name__ == '__main__':
  main()