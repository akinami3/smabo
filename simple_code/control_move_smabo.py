import RPi.GPIO as GPIO 
import readchar

PWM_MIN = 40
PWM_MAX = 255
PWM_STEP = 1

# GPIOピンの設定
# 左モーター用のGPIOピン番号を定義
LEFT_PWM = 21  # 左モータードライバのPWM制御
LEFT_IN1 = 20  # 左モータードライバのIN1
LEFT_IN2 = 16  # 左モータードライバのIN2

# 右モーター用のGPIOピン番号を定
RIGHT_PWM = 22  # 右モータードライバのPWM制御
RIGHT_IN1 = 27  # 右モータードライバのIN1
RIGHT_IN2 = 17  # 右モータードライバのIN2

# 進行方向に関する定数
STOP = 0
FORWARD = 1
BACK = 2
TURN_RIGHT = 3
TURN_LEFT = 4
CURVE_RIGHT_FRONT = 5
CURVE_RIGHT_BACK = 6
CURVE_LEFT_FRONT = 7
CURVE_LEFT_BACK = 8

def control_dc_motor(running_state, motor_power, left_motor_in1, left_motor_in2, right_motor_in1, right_motor_in2, left_motor, right_motor):
    """
    指定された動作状態に基づいてDCモーターを制御

    Parameters
    ----------
    running_state : int
        モーターの動作状態（STOP, FORWARD, BACK, TURN_RIGHT, TURN_LEFT, CURVE_RIGHT_FRONT, CURVE_RIGHT_BACK, CURVE_LEFT_FRONT, CURVE_LEFT_BACK）
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
    if running_state == STOP:
        left_motor.ChangeDutyCycle(0)
        right_motor.ChangeDutyCycle(0)

    if running_state == FORWARD:
        left_motor.ChangeDutyCycle(motor_power)
        right_motor.ChangeDutyCycle(motor_power)
        GPIO.output(left_motor_in1, GPIO.LOW)
        GPIO.output(left_motor_in2, GPIO.HIGH)
        GPIO.output(right_motor_in1, GPIO.HIGH)
        GPIO.output(right_motor_in2, GPIO.LOW)

    if running_state == BACK:
        left_motor.ChangeDutyCycle(motor_power)
        right_motor.ChangeDutyCycle(motor_power)
        GPIO.output(left_motor_in1, GPIO.HIGH)
        GPIO.output(left_motor_in2, GPIO.LOW)
        GPIO.output(right_motor_in1, GPIO.LOW)
        GPIO.output(right_motor_in2, GPIO.HIGH)

    if running_state == TURN_LEFT:
        left_motor.ChangeDutyCycle(motor_power)
        right_motor.ChangeDutyCycle(motor_power)
        GPIO.output(left_motor_in1, GPIO.HIGH)
        GPIO.output(left_motor_in2, GPIO.LOW)
        GPIO.output(right_motor_in1, GPIO.HIGH)
        GPIO.output(right_motor_in2, GPIO.LOW)
    
    if running_state == TURN_RIGHT:
        left_motor.ChangeDutyCycle(motor_power)
        right_motor.ChangeDutyCycle(motor_power)
        GPIO.output(right_motor_in1, GPIO.LOW)
        GPIO.output(right_motor_in2, GPIO.HIGH)
        GPIO.output(left_motor_in1, GPIO.LOW)
        GPIO.output(left_motor_in2, GPIO.HIGH)

    if running_state == CURVE_RIGHT_FRONT:
        left_motor.ChangeDutyCycle(motor_power)
        right_motor.ChangeDutyCycle(0)
        GPIO.output(left_motor_in1, GPIO.LOW)
        GPIO.output(left_motor_in2, GPIO.HIGH)

    if running_state == CURVE_RIGHT_BACK:
        left_motor.ChangeDutyCycle(motor_power)
        right_motor.ChangeDutyCycle(0)
        GPIO.output(left_motor_in1, GPIO.HIGH)
        GPIO.output(left_motor_in2, GPIO.LOW)

    if running_state == CURVE_LEFT_FRONT:
        left_motor.ChangeDutyCycle(0)
        right_motor.ChangeDutyCycle(motor_power)
        GPIO.output(right_motor_in1, GPIO.HIGH)
        GPIO.output(right_motor_in2, GPIO.LOW)

    if running_state == CURVE_LEFT_BACK:
        left_motor.ChangeDutyCycle(0)
        right_motor.ChangeDutyCycle(motor_power)
        GPIO.output(right_motor_in1, GPIO.LOW)
        GPIO.output(right_motor_in2, GPIO.HIGH)


def main():
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
    left_motor_pwm = GPIO.PWM(LEFT_PWM, 100)
    left_motor_pwm.start(0)
    right_motor_pwm = GPIO.PWM(RIGHT_PWM, 100)
    right_motor_pwm.start(0)

    # PWM値を最小値で初期化
    current_pwm = PWM_MIN
    # 初期状態をSTOPで初期化
    current_state = STOP

    try:
        print(f"current pwm: {current_pwm}")
        # 入力したキーに応じて、左右のDCモータを制御
        while True:
            # キー入力受付
            key = readchar.readkey()

            # キー情報に応じて走行を変化させる
            if key == "w": # 前進
                current_state = FORWARD
            elif key == "x": # 後進
                current_state = BACK
            elif key == "a": # 左回転
                current_state = TURN_LEFT
            elif key == "d": # 右回転
                current_state = TURN_RIGHT
            elif key == "s": # 停止
                current_state = STOP
            elif key == "e": # 右カーブ（前進）
                current_state = CURVE_RIGHT_FRONT
            elif key == "c": # 右カーブ（後退）
                current_state = CURVE_RIGHT_BACK
            elif key == "q": # 左カーブ（全身）
                current_state = CURVE_LEFT_FRONT
            elif key == "z": # 左カーブ（後退）
                current_state = CURVE_LEFT_BACK
            elif key == "r": # 速度上昇
                if current_pwm <= PWM_MAX:
                    current_pwm += PWM_STEP
                    print(f"current pwm: {current_pwm}")
            elif key == "v": # 速度低下
                if PWM_MIN <= current_pwm:
                    current_pwm -= PWM_STEP
                    print(f"current pwm: {current_pwm}")

            control_dc_motor(current_state, current_pwm, LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2, left_motor_pwm, right_motor_pwm)

    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == "__main__":
     main()