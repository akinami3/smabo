import RPi.GPIO as GPIO 
import time
import math
import readchar
import threading

# GPIOピンの設定
# 左モーター用のGPIOピン番号を定義
LEFT_PWM = 21  # 左モータードライバのPWM制御
LEFT_IN1 = 20  # 左モータードライバのIN1
LEFT_IN2 = 16  # 左モータードライバのIN2
LEFT_ENCODER_A = 12  # 左エンコーダのA信号
LEFT_ENCODER_B = 25  # 左エンコーダのB信号

# 右モーター用のGPIOピン番号を定
RIGHT_PWM = 22  # 右モータードライバのPWM制御
RIGHT_IN1 = 27  # 右モータードライバのIN1
RIGHT_IN2 = 17  # 右モータードライバのIN2
RIGHT_ENCODER_A = 24  # 右エンコーダのA信号
RIGHT_ENCODER_B = 23  # 右エンコーダのB信号

# 進行方向に関する定数
STOP = 0
FORWARD = 1
BACK = 2
TURN_RIGHT = 3
TURN_LEFT = 4

# ホイールのサイズに関する定数
WHEEL_DIAMETER = 0.068  # タイヤの直径(m)
WHEEL_TREAD = 0.19  # タイヤ間の距離(m)

# 環境やタイヤの材質などを考慮した定数
CONSTANT_RIGHT_WHEEL = 1.0 
CONSTANT_LEFT_WHEEL = 1.0

# 1回転あたりの距離
ONE_REVOLUTION_DISTANCE_RIGHT = math.pi * WHEEL_DIAMETER * CONSTANT_LEFT_WHEEL
ONE_REVOLUTION_DISTANCE_LEFT = math.pi * WHEEL_DIAMETER * CONSTANT_RIGHT_WHEEL

# エンコーダに関する定数を定義
PULSE_PER_REAOLUTION = 295  # 1回転あたりのパルス数

# エンコーダのカウンターの値を保存するための変数
left_pulse_counter = 0  # 左エンコーダのカウント数
right_pulse_counter = 0  # 右エンコーダのカウント数
last_left_pulse_counter = 0  # 前回の左エンコーダのカウント数
last_right_pulse_counter = 0  # 前回の右エンコーダのカウント数

# エンコーダの信号が変わったときに呼び出されるコールバック関数
# 左車輪
def left_encoder_callback(channel):
    global left_pulse_counter 
    state = GPIO.input(LEFT_ENCODER_A)

    # エンコーダが回転方向に応じてカウンタを増減
    if GPIO.input(LEFT_ENCODER_B) == state:  # エンコーダが正の方向に回転している場合
        left_pulse_counter += 1
    else:  # エンコーダが負の方向に回転している場合
        left_pulse_counter -= 1

# 右車輪
def right_encoder_callback(channel):
    global right_pulse_counter
    state = GPIO.input(RIGHT_ENCODER_A)

    # エンコーダが回転方向に応じてカウンタを増減
    if GPIO.input(RIGHT_ENCODER_B) != state:  # エンコーダが正の方向に回転している場合
        right_pulse_counter += 1
    else:  # エンコーダが負の方向に回転している場合
        right_pulse_counter -= 1

def control_dc_motor(running_state, motor_power, left_motor_in1, left_motor_in2, right_motor_in1, right_motor_in2, left_motor, right_motor):
    """
    指定された動作状態に基づいてDCモーターを制御

    Parameters
    ----------
    running_state : int
        モーターの動作状態（STOP, FORWARD, BACK, TURN_RIGHT, TURN_LEFT）
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

def calculate_odometry_from_pulse_counts(x, y, theta):
    """
    エンコーダのパルスカウントからロボットのオドメトリ（位置と向き）を計算

    Parameters
    ----------
    x : float
        現在のx座標
    y : float
        現在のy座標
    theta : float
        現在の向き（ラジアン）

    Returns
    -------
    float
        更新されたx座標
    float
        更新されたy座標
    float
        更新された向き（ラジアン）
    """
    global last_left_pulse_counter, last_right_pulse_counter
    global left_pulse_counter, right_pulse_counter

    # パルスカウントの変化量
    pulse_count_difference_left = left_pulse_counter - last_left_pulse_counter
    pulse_count_difference_right = right_pulse_counter - last_right_pulse_counter

    # 現在のエンコーダのカウントを記録
    last_left_pulse_counter = left_pulse_counter
    last_right_pulse_counter = right_pulse_counter

    # 左右のホイールの回転数を計算
    left_revolutions = pulse_count_difference_left / PULSE_PER_REAOLUTION
    right_revolutions = pulse_count_difference_right / PULSE_PER_REAOLUTION

    # 左右のホイールが移動した距離を計算
    left_distance = left_revolutions * ONE_REVOLUTION_DISTANCE_LEFT
    right_distance = right_revolutions * ONE_REVOLUTION_DISTANCE_RIGHT
    # 左右の移動距離から、平均移動距離を計算
    average_distance = (right_distance + left_distance) / 2

    # ロボットの位置、向き(x,y,theta)を更新
    theta += math.atan2(right_distance - left_distance, WHEEL_TREAD)
    x += average_distance * math.cos(theta)
    y += average_distance * math.sin(theta)

    return x, y, theta

def calc_odometry():
    # ロボットの初期位置と向きを設定
    x = 0.0
    y = 0.0
    theta = 0.0
    while True:
        x, y, theta = calculate_odometry_from_pulse_counts(x, y, theta)
        print("x:", x)
        print("y:", y)
        print("theta:", math.degrees(theta)) 
        print("----------------")
        time.sleep(0.1)

GPIO.setmode(GPIO.BCM) # GPIOのピン番号で設定するモード

# 左モーターのGPIOピンを設定
GPIO.setup(LEFT_IN1, GPIO.OUT)
GPIO.setup(LEFT_IN2, GPIO.OUT)
GPIO.setup(LEFT_PWM, GPIO.OUT)
# 左エンコーダのGPIOピンを入力モードで設定し、プルアップ抵抗を有効化
GPIO.setup(LEFT_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LEFT_ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# 右モーターのGPIOピンを設定
GPIO.setup(RIGHT_IN1, GPIO.OUT)
GPIO.setup(RIGHT_IN2, GPIO.OUT)
GPIO.setup(RIGHT_PWM, GPIO.OUT)
# 右エンコーダのGPIOピンを入力モードで設定し、プルアップ抵抗を有効化
GPIO.setup(RIGHT_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RIGHT_ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# 左右のエンコーダの割り込みを設定。エンコーダのA信号が変化したときにコールバック関数が呼び出される。
GPIO.add_event_detect(LEFT_ENCODER_A, GPIO.BOTH, callback=left_encoder_callback)
GPIO.add_event_detect(RIGHT_ENCODER_A, GPIO.BOTH, callback=right_encoder_callback)

# モーターのPWMを設定し、PWM信号を開始
left_motor_pwm = GPIO.PWM(LEFT_PWM, 100)
left_motor_pwm.start(0)
right_motor_pwm = GPIO.PWM(RIGHT_PWM, 100)
right_motor_pwm.start(0)

# オドメトリの計算を別スレッドで実施
calc_odometry_thread = threading.Thread(target=calc_odometry)
calc_odometry_thread.start()

try:
    # 入力したキーに応じて、左右のDCモータを制御
    while True:
        key = readchar.readkey()
        if key == "w": # 前進
            control_dc_motor(FORWARD, 80, LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2, left_motor_pwm, right_motor_pwm)
        elif key == "x": # 後進
            control_dc_motor(BACK, 80, LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2, left_motor_pwm, right_motor_pwm)
        elif key == "a": # 左回転
            control_dc_motor(TURN_LEFT, 80, LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2, left_motor_pwm, right_motor_pwm)
        elif key == "d": # 右回転
            control_dc_motor(TURN_RIGHT, 80, LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2, left_motor_pwm, right_motor_pwm)
        elif key == "s": # 停止
            control_dc_motor(STOP, 0, LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2, left_motor_pwm, right_motor_pwm)

except KeyboardInterrupt:
    GPIO.cleanup()
    calc_odometry_thread.join()