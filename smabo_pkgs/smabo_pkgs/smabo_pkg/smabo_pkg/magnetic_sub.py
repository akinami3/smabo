import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from smabo_messages.msg import Magnetic

class MagneticSub(Node):
    def __init__(self):
        super().__init__('magnetic_sub')
        self.magnetic = Magnetic()

        # subscriber 設定
        self.magnetic_sub = self.create_subscription(
            Magnetic,
            '/magnetic',
            self.magnetic_callback,
            qos_profile_sensor_data # センサーデータに適切なqos_profileを設定
        )

    def magnetic_callback(self, msg):
        self.magnetic = msg # センサ値をメンバ変数に格納
        self.get_logger().info(f"magnetic: {msg.magnetic}")


def main(args=None):
    rclpy.init(args=args) # 初期化
    magnetic_sub = MagneticSub() # ノード生成
    rclpy.spin(magnetic_sub)
    magnetic_sub.destroy_node() # ノード破壊
    rclpy.shutdown() # 終了

if __name__ == '__main__':
    main()