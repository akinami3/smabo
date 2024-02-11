import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from smabo_messages.msg import Expression

class ImpressionPub(Node):

    def __init__(self):
        super().__init__("impression_pub")
        # publisherの定義
        # QOSプロファイルの設定　参考：https://docs.ros.org/en/eloquent/Concepts/About-Quality-of-Service-Settings.html
        qos_profile = QoSProfile(depth=10) # キューサイズの設定（深いほどメッセージを多く保持できるが、ネットワークやsubscriberへの負担が大きくなる）
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE # 通信の信頼性をreliability（確実にメッセージを届ける）に設定
        self.impression_publisher = self.create_publisher(Expression, '/change_expression', qos_profile) # self.create_publisher(メッセージの型, トピック名, qos_profile)

    def publish_impression(self):
        while True:
            try:
                msg = Expression() # メッセージ生成
                self.get_logger().info("----------------------------")
                self.get_logger().info("Please expression number")
                self.get_logger().info(f"{Expression.NORMAL}:Normal, {Expression.SAD}:Sad, {Expression.GLAD}:Glad, {Expression.ANGRY}:Angry, {Expression.SLEEPY}:Sleepy, {Expression.SLEEP}:Sleep")
                msg.expression = int(input()) # キーボードからの入力をメッセージの変数「expression」に格納
                self.impression_publisher.publish(msg) # メッセージをpublish
                self.get_logger().info("published")
            except KeyboardInterrupt:
                self.get_logger().info("end")
                break

def main(args=None):
    rclpy.init(args=args) # 初期化
    impression_pub_node = ImpressionPub() # ノード生成
    impression_pub_node.publish_impression()
    impression_pub_node.destroy_node() # ノード破壊
    rclpy.shutdown() # 終了

if __name__ == '__main__':
    main()