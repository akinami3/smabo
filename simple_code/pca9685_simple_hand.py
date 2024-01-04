import Adafruit_PCA9685

# 定数
# SET_FREQ, STEP, MAX_PLUSE, MIN_PULSEは使用するサーボモータの仕様に合わせてください。
SET_FREQ = 50  # 周波数設定 (Hz)
STEP = 4096  # 分解能 (ステップ数)
MAX_PULSE = 2.4  # 90度のパルス間隔 (ms)
MIN_PULSE = 0.5  # -90度のパルス間隔 (ms)
CENTER_PULSE = (MAX_PULSE - MIN_PULSE) / 2 + MIN_PULSE  # 0度のパルス間隔 (ms)
PULSE_PER_DEGREE = (MAX_PULSE - MIN_PULSE) / 180  # 1度あたりのパルス間隔 (ms)

def convert_deg_to_pulse(deg, freq):
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

def set_servo_angle(pwm, pin, angle, freq):
    """
    指定したピンに接続されたサーボモーターを特定の角度に設定します。

    Parameters
    ----------
    pwm : Adafruit_PCA9685.PCA9685
        Adafruit PCA9685のインスタンス。
    pin : int
        ピン番号。
    angle : int
        サーボモーターに設定する角度 (-90から90)。
    freq : int
        PWMの周波数 (Hz)。

    """
    pwm.set_pwm(pin, 0, convert_deg_to_pulse(angle, freq))

def main():
    # Adafruit_PCA9685の初期設定
    pwm = Adafruit_PCA9685.PCA9685()
    pwm.set_pwm_freq(SET_FREQ)

    # 右腕、左腕のピンを設定
    right_hand_pin = 6
    left_hand_pin = 7

    # 初期角度に設定
    initial_deg = 0
    set_servo_angle(pwm, left_hand_pin, initial_deg, SET_FREQ)
    set_servo_angle(pwm, right_hand_pin, initial_deg, SET_FREQ)

    try:
        while True:
            print("腕の角度を指定してください（-90度～90度）")
            deg = int(input())
            set_servo_angle(pwm, right_hand_pin, deg, SET_FREQ)
            set_servo_angle(pwm, left_hand_pin, -deg, SET_FREQ)

    except KeyboardInterrupt:
        set_servo_angle(pwm, left_hand_pin, 0, SET_FREQ)
        set_servo_angle(pwm, right_hand_pin, 0, SET_FREQ)
        print("プログラムを終了します。")

if __name__ == "__main__":
    main()