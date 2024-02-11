import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import RPi.GPIO as GPIO 
from smabo_messages.msg import DcMotor

# GPIOピンの設定
# 左モーター用のGPIOピン番号を定義
LEFT_PWM = 21  # 左モータードライバのPWM制御
LEFT_IN1 = 20  # 左モータードライバのIN1
LEFT_IN2 = 16  # 左モータードライバのIN2

# 右モーター用のGPIOピン番号を定
RIGHT_PWM = 22  # 右モータードライバのPWM制御
RIGHT_IN1 = 27  # 右モータードライバのIN1
RIGHT_IN2 = 17  # 右モータードライバのIN2

class MoveRobotDcController(Node):
    def __init__(self):
        super().__init__('move_robot_dc_controller')

        # QOSプロファイルの設定　参考：https://docs.ros.org/en/eloquent/Concepts/About-Quality-of-Service-Settings.html
        qos_profile = QoSProfile(depth=10) # キューサイズの設定（深いほどメッセージを多く保持できるが、ネットワークやsubscriberへの負担が大きくなる）
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE # 通信の信頼性をreliability（確実にメッセージを届ける）に設定
        
        # subscriber 設定
        self.control_dc_motor_sub = self.create_subscription(
            DcMotor,
            '/dc_motor',
            self.dc_motor_callback,
            qos_profile 
        )

        # GPIO初期化
        GPIO.setmode(GPIO.BCM) # GPIOのピン番号で設定するモード

        # 左モーターのGPIOピンを設定
        GPIO.setup(LEFT_IN1, GPIO.OUT)
        GPIO.setup(LEFT_IN2, GPIO.OUT)
        GPIO.setup(LEFT_PWM, GPIO.OUT)

        # 右モーターのGPIOピンを設定
        GPIO.setup(RIGHT_IN1, GPIO.OUT)
        GPIO.setup(RIGHT_IN2, GPIO.OUT)
        GPIO.setup(RIGHT_PWM, GPIO.OUT)

        # モーターのPWMを設定し、PWM信号を開始
        self.left_motor_pwm = GPIO.PWM(LEFT_PWM, 100)
        self.left_motor_pwm.start(0)
        self.right_motor_pwm = GPIO.PWM(RIGHT_PWM, 100)
        self.right_motor_pwm.start(0)

        # 初期状態をSTOPで初期化
        self.last_state = DcMotor.STOP   

    def __del__(self):
        GPIO.cleanup()

    def dc_motor_callback(self, msg):
        self.control_dc_motor(msg.running_state, msg.pwm_val, LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2, self.left_motor_pwm, self.right_motor_pwm)

    def control_dc_motor(self, running_state, motor_power, left_motor_in1, left_motor_in2, right_motor_in1, right_motor_in2, left_motor, right_motor):
        """
        指定された動作状態に基づいてDCモーターを制御

        Parameters
        ----------
        running_state : int
            モーターの動作状態（STOP, FORWARD, BACK, TURN_RIGHT, TURN_LEFT, ONLY_LEFT_FRONT, ONLY_LEFT_BACK, ONLY_RIGHT_FRONT, CURVE_LEFT_BACK）
        motor_power : int
            モーターの動作強度（PWM値）
        left_motor_in1 : int
            左モーターのピン番号1
        left_motor_in2 : int
            左モーターのピン番号2
        right_motor_in1 : int
            右モーターのピン番号1
        right_motor_in2 : int
            右モーターのピン番号2
        left_motor : GPIO.PWM
            左モーターのPWMインスタンス
        right_motor : GPIO.PWM
            右モーターのPWMインスタンス
        """
        if running_state == DcMotor.STOP:
            left_motor.ChangeDutyCycle(0)
            right_motor.ChangeDutyCycle(0)

        if running_state == DcMotor.FORWARD:
            left_motor.ChangeDutyCycle(motor_power)
            right_motor.ChangeDutyCycle(motor_power)
            GPIO.output(left_motor_in1, GPIO.LOW)
            GPIO.output(left_motor_in2, GPIO.HIGH)
            GPIO.output(right_motor_in1, GPIO.HIGH)
            GPIO.output(right_motor_in2, GPIO.LOW)

        if running_state == DcMotor.BACK:
            left_motor.ChangeDutyCycle(motor_power)
            right_motor.ChangeDutyCycle(motor_power)
            GPIO.output(left_motor_in1, GPIO.HIGH)
            GPIO.output(left_motor_in2, GPIO.LOW)
            GPIO.output(right_motor_in1, GPIO.LOW)
            GPIO.output(right_motor_in2, GPIO.HIGH)

        if running_state == DcMotor.TURN_LEFT:
            left_motor.ChangeDutyCycle(motor_power)
            right_motor.ChangeDutyCycle(motor_power)
            GPIO.output(left_motor_in1, GPIO.HIGH)
            GPIO.output(left_motor_in2, GPIO.LOW)
            GPIO.output(right_motor_in1, GPIO.HIGH)
            GPIO.output(right_motor_in2, GPIO.LOW)
        
        if running_state == DcMotor.TURN_RIGHT:
            left_motor.ChangeDutyCycle(motor_power)
            right_motor.ChangeDutyCycle(motor_power)
            GPIO.output(right_motor_in1, GPIO.LOW)
            GPIO.output(right_motor_in2, GPIO.HIGH)
            GPIO.output(left_motor_in1, GPIO.LOW)
            GPIO.output(left_motor_in2, GPIO.HIGH)

        if running_state == DcMotor.ONLY_LEFT_FRONT:
            left_motor.ChangeDutyCycle(motor_power)
            right_motor.ChangeDutyCycle(0)
            GPIO.output(left_motor_in1, GPIO.LOW)
            GPIO.output(left_motor_in2, GPIO.HIGH)

        if running_state == DcMotor.ONLY_LEFT_BACK:
            left_motor.ChangeDutyCycle(motor_power)
            right_motor.ChangeDutyCycle(0)
            GPIO.output(left_motor_in1, GPIO.HIGH)
            GPIO.output(left_motor_in2, GPIO.LOW)

        if running_state == DcMotor.ONLY_RIGHT_FRONT:
            left_motor.ChangeDutyCycle(0)
            right_motor.ChangeDutyCycle(motor_power)
            GPIO.output(right_motor_in1, GPIO.HIGH)
            GPIO.output(right_motor_in2, GPIO.LOW)

        if running_state == DcMotor.ONLY_RIGHT_BACK:
            left_motor.ChangeDutyCycle(0)
            right_motor.ChangeDutyCycle(motor_power)
            GPIO.output(right_motor_in1, GPIO.LOW)
            GPIO.output(right_motor_in2, GPIO.HIGH)


def main(args=None):
    rclpy.init(args=args) # 初期化
    node = MoveRobotDcController() # ノード生成
    rclpy.spin(node)
    node.destroy_node() # ノード破壊
    rclpy.shutdown() # 終了

if __name__ == '__main__':
    main()
