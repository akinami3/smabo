import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Accel

class AccelSub(Node):
    def __init__(self):
        super().__init__('accel_sub')
        self.accel = Accel()

        # subscriber 設定
        self.accel_sub = self.create_subscription(
            Accel,
            '/accel',
            self.accel_callback,
            qos_profile_sensor_data # センサーデータに適切なqos_profileを設定
        )

    def accel_callback(self, msg):
        self.accel = msg # センサ値をメンバ変数に格納
        self.get_logger().info(f"accel: ({msg.accel.x},{msg.accel.y},{msg.accel.z})")

def main(args=None):
    rclpy.init(args=args) # 初期化
    accel_sub = AccelSub() # ノード生成
    rclpy.spin(accel_sub)
    accel_sub.destroy_node() # ノード破壊
    rclpy.shutdown() # 終了

if __name__ == '__main__':
    main()