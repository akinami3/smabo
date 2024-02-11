std_msgs/Header header

# 走行状態
uint8 running_state

uint8 STOP=0
uint8 FORWARD=1
uint8 BACK=2
uint8 TURN_RIGHT=3
uint8 TURN_LEFT=4
uint8 CURVE_RIGHT_FRONT=5
uint8 CURVE_RIGHT_BACK=6
uint8 CURVE_LEFT_FRONT=7
uint8 CURVE_LEFT_BACK=8

# pwm値
uint8 pwm_val
