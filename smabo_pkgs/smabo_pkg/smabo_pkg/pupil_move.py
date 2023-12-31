import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from smabo_messages.msg import TwoDimensionPosition

class PupilMove(Node):

    def __init__(self):
        super().__init__("pupil_move")
        # publisherの定義
        # QOSプロファイルの設定　参考：https://docs.ros.org/en/eloquent/Concepts/About-Quality-of-Service-Settings.html
        qos_profile = QoSProfile(depth=10) # キューサイズの設定（深いほどメッセージを多く保持できるが、ネットワークやsubscriberへの負担が大きくなる）
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE # 通信の信頼性をreliability（確実にメッセージを届ける）に設定
        self.position_publisher = self.create_publisher(TwoDimensionPosition, '/pupil_move', qos_profile) # self.create_publisher(メッセージの型, トピック名, qos_profile)

    def publish_position(self):
        while True:
            try:
                msg = TwoDimensionPosition() # メッセージ生成
                self.get_logger().info("----------------------------")
                self.get_logger().info("Please expression number")
                msg.min_point.x = 0.0
                msg.min_point.y = 0.0
                msg.max_point.x = 100.0
                msg.max_point.y = 100.0
                self.get_logger().info(f"x (0-100)")
                msg.current_point.x = float(input())
                self.get_logger().info(f"y (0-100)")
                msg.current_point.y = float(input())
                self.position_publisher.publish(msg) # メッセージをpublish
                self.get_logger().info("published")
            except KeyboardInterrupt:
                self.get_logger().info("end")
                break

def main(args=None):
    rclpy.init(args=args) # 初期化
    pupil_move_node = PupilMove() # ノード生成
    pupil_move_node.publish_position()
    pupil_move_node.destroy_node() # ノード破壊
    rclpy.shutdown() # 終了

if __name__ == '__main__':
    main()