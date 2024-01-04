import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from smabo_messages.msg import Servo

class SimpleHandAngleCommander(Node):

    def __init__(self):
        super().__init__("simple_hand_angle_commander")
        # publisherの定義
        # QOSプロファイルの設定　参考：https://docs.ros.org/en/eloquent/Concepts/About-Quality-of-Service-Settings.html
        qos_profile = QoSProfile(depth=10) # キューサイズの設定（深いほどメッセージを多く保持できるが、ネットワークやsubscriberへの負担が大きくなる）
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE # 通信の信頼性をreliability（確実にメッセージを届ける）に設定
        self.servo_publisher = self.create_publisher(Servo, '/servo', qos_profile) # self.create_publisher(メッセージの型, トピック名, qos_profile)

    def publish_servo(self):
        while True:
            try:
                self.get_logger().info("----------------------------")
                self.get_logger().info("Please hand angle number")
                angle = int(input())

                # 右手
                msg = Servo()
                msg.pin_no = 6
                msg.angle = angle
                self.servo_publisher.publish(msg)

                # 左手
                msg = Servo()
                msg.pin_no = 7
                msg.angle = -angle
                self.servo_publisher.publish(msg)

                self.get_logger().info("published")
            except KeyboardInterrupt:
                self.get_logger().info("end")
                break

def main(args=None):
    rclpy.init(args=args) # 初期化
    node = SimpleHandAngleCommander() # ノード生成
    node.publish_servo()
    node.destroy_node() # ノード破壊
    rclpy.shutdown() # 終了

if __name__ == '__main__':
    main()