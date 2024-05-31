import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from smabo_messages.msg import Servo
import readchar
import time

class Link3RobotArmCommander(Node):

    def __init__(self):
        super().__init__("link3_robot_arm_commander")
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        self.servo_publisher = self.create_publisher(Servo, '/servo', qos_profile)

        # 各リンク、グリッパの初期角度を設定
        self.link1_servo_msg = Servo()
        self.link1_servo_msg.pin_no = 5
        self.link1_servo_msg.angle = 0
        self.servo_publisher.publish(self.link1_servo_msg)

        self.link2_servo_msg = Servo()
        self.link2_servo_msg.pin_no = 13
        self.link2_servo_msg.angle = 0
        self.servo_publisher.publish(self.link2_servo_msg)

        self.link3_servo_msg = Servo()
        self.link3_servo_msg.pin_no = 14
        self.link3_servo_msg.angle = 0
        self.servo_publisher.publish(self.link3_servo_msg)

        self.gripper_servo_msg = Servo()
        self.gripper_servo_msg.pin_no = 15
        self.gripper_servo_msg.angle = 0
        self.servo_publisher.publish(self.gripper_servo_msg)

        time.sleep(0.1)

        self.angle_step = 1 # 一度のキー入力で何度回転させるか

    def publish_servo(self):
        self.get_logger().info(f"=================Please key ====================")
        self.get_logger().info(f"link1 z: angle+{self.angle_step}, x: angle-{self.angle_step} ")
        self.get_logger().info(f"link2 q: angle+{self.angle_step}, a: angle-{self.angle_step} ")
        self.get_logger().info(f"link3 w: angle+{self.angle_step}, s: angle-{self.angle_step} ")
        self.get_logger().info(f"gripper e: open, d: close")
        self.get_logger().info(f"================================================")
        while True:
            try:
                key = readchar.readkey()
                # zキーを入力したら、link1を正方向に回転
                if key == "z":
                    if self.link1_servo_msg.angle < 90:
                        self.link1_servo_msg.angle += self.angle_step
                        self.servo_publisher.publish(self.link1_servo_msg)
                        self.get_logger().info(f"Published angle: {self.link1_servo_msg.angle}")
                    else:
                        # 最大角度より大きい角度に回転させようとしたら警告を表示
                        self.get_logger().warn("Cannot rotate further to the left")
                # xキーを入力したら、link1を逆方向に回転
                elif key == "x":
                    if -90 < self.link1_servo_msg.angle:
                        self.link1_servo_msg.angle -= self.angle_step
                        self.servo_publisher.publish(self.link1_servo_msg)
                        self.get_logger().info(f"Published angle: {self.link1_servo_msg.angle}")
                    else:
                        # 最低角度未満の角度に回転させようとしたら警告を表示
                        self.get_logger().warn("Cannot rotate further to the right")
                # qキーを入力したら、link2を正方向に回転
                elif key == "q":
                    if self.link2_servo_msg.angle < 90:
                        self.link2_servo_msg.angle += self.angle_step
                        self.servo_publisher.publish(self.link2_servo_msg)
                        self.get_logger().info(f"Published angle: {self.link2_servo_msg.angle}")
                    else:
                        # 最大角度より大きい角度に回転させようとしたら警告を表示
                        self.get_logger().warn("Cannot rotate further to the left")
                # aキーを入力したら、link2を逆方向に回転
                elif key == "a":
                    if -90 < self.link2_servo_msg.angle:
                        self.link2_servo_msg.angle -= self.angle_step
                        self.servo_publisher.publish(self.link2_servo_msg)
                        self.get_logger().info(f"Published angle: {self.link2_servo_msg.angle}")
                    else:
                        # 最低角度未満の角度に回転させようとしたら警告を表示
                        self.get_logger().warn("Cannot rotate further to the right")
                # wキーを入力したら、link3を正方向に回転
                elif key == "w":
                    if self.link3_servo_msg.angle < 90:
                        self.link3_servo_msg.angle += self.angle_step
                        self.servo_publisher.publish(self.link3_servo_msg)
                        self.get_logger().info(f"Published angle: {self.link3_servo_msg.angle}")
                    else:
                        # 最大角度より大きい角度に回転させようとしたら警告を表示
                        self.get_logger().warn("Cannot rotate further to the left")
                # sキーを入力したら、link3を逆方向に回転
                elif key == "s":
                    if -90 < self.link3_servo_msg.angle:
                        self.link3_servo_msg.angle -= self.angle_step
                        self.servo_publisher.publish(self.link3_servo_msg)
                        self.get_logger().info(f"Published angle: {self.link3_servo_msg.angle}")
                    else:
                        # 最低角度未満の角度に回転させようとしたら警告を表示
                        self.get_logger().warn("Cannot rotate further to the right")
                # eキーを入力したら、gripperを閉じる
                elif key == "e":
                    self.gripper_servo_msg.angle = 90
                    self.servo_publisher.publish(self.gripper_servo_msg)
                    self.get_logger().info(f"Published angle: {self.gripper_servo_msg.angle}")
                # dキーを入力したら、gripperを開く
                elif key == "d":
                    self.gripper_servo_msg.angle = 0
                    self.servo_publisher.publish(self.gripper_servo_msg)
                    self.get_logger().info(f"Published angle: {self.gripper_servo_msg.angle}")
                else:
                    continue
            except KeyboardInterrupt:
                self.get_logger().info("End")
                break

def main(args=None):
    rclpy.init(args=args)
    node = Link3RobotArmCommander()
    node.publish_servo()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()