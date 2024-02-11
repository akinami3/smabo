import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from smabo_messages.msg import DcMotor
import readchar

PWM_MIN = 40
PWM_MAX = 255
PWM_STEP = 1

class MoveRobotCommander(Node):

    def __init__(self):
        super().__init__("move_robot_commander")
        # publisherの定義
        # QOSプロファイルの設定　参考：https://docs.ros.org/en/eloquent/Concepts/About-Quality-of-Service-Settings.html
        qos_profile = QoSProfile(depth=10) # キューサイズの設定（深いほどメッセージを多く保持できるが、ネットワークやsubscriberへの負担が大きくなる）
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE # 通信の信頼性をreliability（確実にメッセージを届ける）に設定
        self.dc_motor_publisher = self.create_publisher(DcMotor, '/dc_motor', qos_profile) # self.create_publisher(メッセージの型, トピック名, qos_profile)

        # PWM値を最小値で初期化
        self.current_pwm = PWM_MIN
        # 初期状態をSTOPで初期化
        self.current_state = DcMotor.STOP

    def publish_dc_motor(self):
        while True:
            try:
                print(f"current pwm: {self.current_pwm}")
                # 入力したキーに応じて、左右のDCモータを制御
                while True:
                    # キー入力受付
                    key = readchar.readkey()

                    # キー情報に応じて走行を変化させる
                    if key == "w": # 前進
                        self.current_state = DcMotor.FORWARD
                    elif key == "x": # 後進
                        self.current_state = DcMotor.BACK
                    elif key == "a": # 左回転
                        self.current_state = DcMotor.TURN_LEFT
                    elif key == "d": # 右回転
                        self.current_state = DcMotor.TURN_RIGHT
                    elif key == "s": # 停止
                        self.current_state = DcMotor.STOP
                    elif key == "e": # 右カーブ（前進）
                        self.current_state = DcMotor.ONLY_LEFT_FRONT
                    elif key == "c": # 右カーブ（後退）
                        self.current_state = DcMotor.ONLY_LEFT_BACK
                    elif key == "q": # 左カーブ（全身）
                        self.current_state = DcMotor.ONLY_RIGHT_FRONT
                    elif key == "z": # 左カーブ（後退）
                        self.current_state = DcMotor.ONLY_RIGHT_BACK
                    elif key == "r": # 速度上昇
                        if self.current_pwm <= PWM_MAX:
                            self.current_pwm += PWM_STEP
                            print(f"current pwm: {self.current_pwm}")
                    elif key == "v": # 速度低下
                        if PWM_MIN <= self.current_pwm:
                            self.current_pwm -= PWM_STEP
                            print(f"current pwm: {self.current_pwm}")
                    
                    # publish
                    msg = DcMotor()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.running_state = self.current_state
                    msg.pwm_val = self.current_pwm
                    self.dc_motor_publisher.publish(msg)

            except KeyboardInterrupt:
                self.get_logger().info("end")
                break

def main(args=None):
    rclpy.init(args=args) # 初期化
    node = MoveRobotCommander() # ノード生成
    node.publish_dc_motor()
    node.destroy_node() # ノード破壊
    rclpy.shutdown() # 終了

if __name__ == '__main__':
    main()