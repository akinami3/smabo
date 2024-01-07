import RPi.GPIO as GPIO
import time

# GPIOピンの設定
ENCODER_A = 12  # エンコーダのA信号
ENCODER_B = 25  # エンコーダのB信号

# 1回転あたりのパルス数（使用するエンコーダによって異なる）
PULSE_PER_REAOLUTION = 295

# エンコーダのパルスのカウンタ
pulse_counter = 0

# エンコーダ読み取りのコールバック関数
def encoder_callback(channel):
    global pulse_counter
    state = GPIO.input(ENCODER_A)

    # エンコーダーが正の方向に回転
    if GPIO.input(ENCODER_B) == state:
        pulse_counter += 1
    else: # エンコーダーが負の方向に回転
        pulse_counter -= 1
    print("encoder counter:", pulse_counter)
    print("current deg:", (pulse_counter/PULSE_PER_REAOLUTION)*360)

# GPIO初期化
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# エンコーダの割り込み設定
GPIO.add_event_detect(ENCODER_A, GPIO.BOTH, callback=encoder_callback)

try:
    while True:
        time.sleep(0.01)

except KeyboardInterrupt:
    GPIO.cleanup()