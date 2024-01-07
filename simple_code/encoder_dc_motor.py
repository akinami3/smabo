import RPi.GPIO as GPIO
import time

# GPIOピンの設定
LEFT_IN1 = 20  # モータードライバのIN1
LEFT_IN2 = 16  # モータードライバのIN2
LEFT_PWM = 21  # モータードライバのPWM
LEFT_ENCODER_A = 12  # エンコーダのA信号
LEFT_ENCODER_B = 25  # エンコーダのB信号

# エンコーダ関連の変数
encoder_counter = 0
last_state = None

# エンコーダ読み取りのコールバック関数
def encoder_callback(channel):
    global encoder_counter, last_state
    state = GPIO.input(LEFT_ENCODER_A)
    if state != last_state:
        if GPIO.input(LEFT_ENCODER_B) != state:
            encoder_counter += 1
        else:
            encoder_counter -= 1
        last_state = state

# GPIO初期化
GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_IN1, GPIO.OUT)
GPIO.setup(LEFT_IN2, GPIO.OUT)
GPIO.setup(LEFT_PWM, GPIO.OUT)
GPIO.setup(LEFT_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LEFT_ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# エンコーダの割り込み設定
GPIO.add_event_detect(LEFT_ENCODER_A, GPIO.BOTH, callback=encoder_callback)

# PWM設定
pwm = GPIO.PWM(LEFT_PWM, 100)  # 100 Hz
pwm.start(0)

try:
    # モーターを時計回りに動かす
    GPIO.output(LEFT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(100)  # 速度調整 (0-100)

    # 5秒間動かす
    while True:
        print(encoder_counter)
        if encoder_counter == 135:
            break

    # モーター停止
    pwm.ChangeDutyCycle(0)
    
    # エンコーダのカウントを表示
    print("Encoder Count:", encoder_counter)

finally:
    # GPIOクリーンアップ
    pwm.stop()
    GPIO.cleanup()
