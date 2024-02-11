import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from smabo_messages.msg import Servo
import Adafruit_PCA9685

# 定数
# SET_FREQ, STEP, MAX_PLUSE, MIN_PULSEは使用するサーボモータの仕様に合わせてください。
SET_FREQ = 50  # 周波数設定 (Hz)
STEP = 4096  # 分解能 (ステップ数)
MAX_PULSE = 2.4  # 90度のパルス間隔 (ms)
MIN_PULSE = 0.5  # -90度のパルス間隔 (ms)
CENTER_PULSE = (MAX_PULSE - MIN_PULSE) / 2 + MIN_PULSE  # 0度のパルス間隔 (ms)
PULSE_PER_DEGREE = (MAX_PULSE - MIN_PULSE) / 180  # 1度あたりのパルス間隔 (ms)

class Pca9685Controller(Node):
    def __init__(self):
        super().__init__('pca9685_controller')

        # Adafruit_PCA9685の初期設定
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(SET_FREQ)

        # QOSプロファイルの設定　参考：https://docs.ros.org/en/eloquent/Concepts/About-Quality-of-Service-Settings.html
        qos_profile = QoSProfile(depth=10) # キューサイズの設定（深いほどメッセージを多く保持できるが、ネットワークやsubscriberへの負担が大きくなる）
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE # 通信の信頼性をreliability（確実にメッセージを届ける）に設定
        
        # subscriber 設定
        self.control_servo_sub = self.create_subscription(
            Servo,
            '/servo',
            self.servo_callback,
            qos_profile
        )
    def __del__(self):
        self.pwm.set_all_pwm(0,0)

    def servo_callback(self, msg):
        self.get_logger().info(f"pin_no:{msg.pin_no}, deg:{msg.angle}")
        self.set_servo_angle(self.pwm, msg.pin_no, msg.angle, SET_FREQ)

    def convert_deg_to_pulse(self, deg, freq):
        """
        角度をPWMコントローラー用のパルスに変換します。

        Parameters
        ----------
        deg : int
            サーボモーターに設定する角度 (-90から90)。
        freq : int
            PWMの周波数 (Hz)。

        Returns
        -------
        int
            PWMコントローラー用のパルス幅。

        """
        deg_pulse = CENTER_PULSE + deg * PULSE_PER_DEGREE  # 要求角度のパルス間隔 (ms)
        deg_num = int(deg_pulse / (1.0 / freq * 1000 / STEP))  # PCA9685に渡す値を算出
        return deg_num

    def set_servo_angle(self, pwm, pin, angle, freq):
        """
        指定したGPIOピンに接続されたサーボモーターを特定の角度に設定します。

        Parameters
        ----------
        pwm : Adafruit_PCA9685.PCA9685
            Adafruit PCA9685のインスタンス。
        pin : int
            GPIOピン番号。
        angle : int
            サーボモーターに設定する角度 (-90から90)。
        freq : int
            PWMの周波数 (Hz)。

        """
        pwm.set_pwm(pin, 0, self.convert_deg_to_pulse(angle, freq))

def main(args=None):
    rclpy.init(args=args) # 初期化
    node = Pca9685Controller() # ノード生成
    rclpy.spin(node)
    node.destroy_node() # ノード破壊
    rclpy.shutdown() # 終了

if __name__ == '__main__':
    main()
