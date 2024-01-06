import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time

class UltrasonicSensor(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor')
        # パラメータの宣言とデフォルト値の設定
        # なお、パラメータを変更して実行したい場合は、下記コマンドのように実行
        # $ ros2 run smabo_pkg ultrasonic_sensor --ros-args -p trig_pin:=＜新しいピン番号＞ echo_pin:=＜新しいピン番号＞ temperature:=＜新しい室温＞
        self.declare_parameter('trig_pin', 26)
        self.declare_parameter('echo_pin', 19)
        self.declare_parameter('temperature', 25) # 室温によって距離の結果が少し変わる

        # パラメータの取得
        self.trig_pin = self.get_parameter('trig_pin').get_parameter_value().integer_value
        self.echo_pin = self.get_parameter('echo_pin').get_parameter_value().integer_value
        self.temperature = self.get_parameter('temperature').get_parameter_value().double_value

        # publisher
        self.publisher = self.create_publisher(Range, 'ultrasonic_range', qos_profile_sensor_data) # センサーデータに適したQoSプロファイルを指定
        
        # timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        GPIO.setmode(GPIO.BCM) # ピンを「GPIOの数字」で指定するモードに設定
        GPIO.setup(self.trig_pin, GPIO.OUT) # trig_pinを「出力モード」に設定
        GPIO.setup(self.echo_pin, GPIO.IN) # echo_pinを「入力モード」に設定
        # 注意：GPIOに5Vを入力するとラズパイが壊れるので，echo_pinは抵抗を使って電圧を下げるように配線を組むように

    def calculate_distance(self):
        """
        距離を計算

        Returns:
            float: 計算された距離（cm）。距離が測定できない場合はNoneを返す
        """
        v = 33150 + 60 * self.temperature

        # trig_pinから0.00001秒の間、信号を出力し、超音波を発信
        GPIO.output(self.trig_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, GPIO.LOW)

        t1, t2 = None, None # 時間計測用の変数を初期化

        # 超音波が返ってくるまでは，echo_pinはLOW
        while GPIO.input(self.echo_pin) == GPIO.LOW:
            t1 = time.time()

        # 超音波が返ってくることを検知すると，echo_pinがHIGHになる
        while GPIO.input(self.echo_pin) == GPIO.HIGH:
            t2 = time.time()

        # 距離を計算
        if t1 and t2:
            t = t2 - t1 # EchoピンがHIGHであった時間
            return v * t / 2 # 距離（cm）
        return None

    def timer_callback(self):
        # 超音波センサから取得した距離情報をpublish
        distance = self.calculate_distance()
        if distance is not None:
            range_msg = Range() # メッセージ生成

            # 使用するセンサー（今回の場合、hc-sr04）に応じて、情報格納
            range_msg.radiation_type = Range.ULTRASOUND # 超音波センサなので、ULTRASOUND(0)に設定
            range_msg.field_of_view = 0.3 # hc-sr04の視野角は約15度～20度なので、その中間の値（単位はradian）を格納
            range_msg.min_range = 0.02 # 最小検出距離[m]
            range_msg.max_range = 0.4 # 最大検出距離[m]

            range_msg.range = distance / 100  # mに変換した距離情報を格納
            range_msg.header.stamp = self.get_clock().now().to_msg() # 現在時刻の情報を格納
            self.publisher.publish(range_msg)
            self.get_logger().info(f"distance: {range_msg.range} m")


def main(args=None):
    rclpy.init(args=args)
    ultrasonic_sensor_node = UltrasonicSensor()
    try:
        rclpy.spin(ultrasonic_sensor_node)
    except KeyboardInterrupt:
        ultrasonic_sensor_node.destroy_node()
        GPIO.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

