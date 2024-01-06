import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from smabo_messages.msg import Servo
import readchar
import time

class HeadArrowKeyCommander(Node):

    def __init__(self):
        super().__init__("head_arrow_key_commander")
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        self.servo_publisher = self.create_publisher(Servo, '/servo', qos_profile)

        # 初期角度を配信
        self.msg = Servo()
        self.msg.pin_no = 5
        self.msg.angle = 0
        self.servo_publisher.publish(self.msg)
        time.sleep(0.1)

        self.angle_step = 1 # 一度のキー入力で何度回転させるか

    def publish_servo(self):
        self.get_logger().info("Press Left or Right arrow key to adjust angle")
        while True:
            try:
                key = readchar.readkey()
                # zキーを入力したら、左方向に回転
                if key == "z":
                    if self.msg.angle < 90:
                        self.msg.angle += self.angle_step
                        self.servo_publisher.publish(self.msg)
                        self.get_logger().info(f"Published angle: {self.msg.angle}")
                    else:
                        # 最大角度より大きい角度に回転させようとしたら警告を表示
                        self.get_logger().warn("Cannot rotate further to the left")
                # xキーを入力したら、左方向に回転
                elif key == "x":
                    if -90 < self.msg.angle:
                        self.msg.angle -= self.angle_step
                        self.servo_publisher.publish(self.msg)
                        self.get_logger().info(f"Published angle: {self.msg.angle}")
                    else:
                        # 最低角度未満の角度に回転させようとしたら警告を表示
                        self.get_logger().warn("Cannot rotate further to the right")
                else:
                    continue
            except KeyboardInterrupt:
                self.get_logger().info("End")
                break

def main(args=None):
    rclpy.init(args=args)
    node = HeadArrowKeyCommander()
    node.publish_servo()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
