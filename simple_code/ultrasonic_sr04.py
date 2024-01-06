import RPi.GPIO as GPIO
import time

def calculate_distance(trig_pin, echo_pin, temperature):
    """
    距離を計算

    Parameters
    ----------
    trig_pin : int
        トリガーピンのGPIO番号
    echo_pin : int
        エコーピンのGPIO番号
    temperature : float
        湿度

    Returns
    -------
    float
        計算された距離（cm）。距離が測定できない場合はNoneを返す
    """
    # 音速を計算（cm/s）
    v = 33150 + 60 * temperature

    # trig_pinから0.00001秒の間、信号を出力し、超音波を発信
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)  # 10マイクロ秒待機
    GPIO.output(trig_pin, GPIO.LOW)

    t1, t2 = None, None  # 時間計測用の変数を初期化

    # 超音波が返ってくるまでは，echo_pinはLOW
    while GPIO.input(echo_pin) == GPIO.LOW:
        t1 = time.time()

    # 超音波が返ってくることを検知すると，echo_pinがHIGHになる
    while GPIO.input(echo_pin) == GPIO.HIGH:
        t2 = time.time()

    # 距離を計算
    if t1 and t2:
        t = t2 - t1  # EchoピンがHIGHであった時間
        return v * t / 2  # 距離（cm）

    return None

def main():
    trig_pin = 26  # トリガーピンのGPIO番号
    echo_pin = 19  # エコーピンのGPIO番号
    temperature = 25  # 湿度（湿度によって距離の結果が少し変わるので、適宜変更すること）

    # GPIOピンのセットアップ
    GPIO.setmode(GPIO.BCM) # ピンを「GPIOの数字」で指定するモードに設定
    GPIO.setup(trig_pin, GPIO.OUT) # trig_pinを「出力モード」に設定
    GPIO.setup(echo_pin, GPIO.IN) # echo_pinを「入力モード」に設定
    # 注意：GPIOに5Vを入力するとラズパイが壊れるので，echo_pinは抵抗を使って電圧を下げるように配線を組むように

    try:
        while True:
            # 距離を計算
            distance = calculate_distance(trig_pin, echo_pin, temperature)

            # 距離が測定された場合、その値を表示
            if distance is not None:
                print(f"{distance:.2f} cm")

            # 0.1秒待機
            time.sleep(0.1)

    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
