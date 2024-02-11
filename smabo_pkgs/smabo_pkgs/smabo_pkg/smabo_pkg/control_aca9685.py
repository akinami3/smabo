import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class ControlAdafruitPca9685(Node):
    def __init__(self):
        super().__init__('control_adafruit_pca9685')

        # subscriber 設定
        self.control_servo_sub = self.create_subscription(
            Int8,
            '/gyro',
            self.gyro_callback,
            qos_profile_sensor_data # センサーデータに適切なqos_profileを設定
        )

    def gyro_callback(self, msg):
        self.gyro = msg # センサ値をメンバ変数に格納
        self.get_logger().info(f"gyro: ({msg.gyro.x},{msg.gyro.y},{msg.gyro.z}, {msg.gyro.z})")

def main(args=None):
    rclpy.init(args=args) # 初期化
    gyro_sub = ControlAdafruitPca9685() # ノード生成
    rclpy.spin(gyro_sub)
    gyro_sub.destroy_node() # ノード破壊
    rclpy.shutdown() # 終了

if __name__ == '__main__':
    main()