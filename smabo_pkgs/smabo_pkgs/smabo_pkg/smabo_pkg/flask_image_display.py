from flask import Flask, Response
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import ipget

app = Flask(__name__) 

class FlaskImageDisplay(Node):
    def __init__(self):
        super().__init__('flask_image_display')
        
        # subscriber
        self.subscription = self.create_subscription(
            Image,
            '/image', 
            self.listener_callback,
            qos_profile_sensor_data  # センサーデータのためのQoSプロファイルを指定
        )
        self.bridge = CvBridge()  # CvBridgeインスタンスを作成
        self.last_frame = None  # 最後に受け取ったフレームを保持する変数

    def listener_callback(self, msg):
        # ROSメッセージをOpenCV形式に変換
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # ROSメッセージをOpenCVイメージに変換
        ret, buffer = cv2.imencode('.jpg', cv_image)  # OpenCVイメージをJPEG形式にエンコード
        self.last_frame = buffer.tobytes()  # エンコードされたデータをバイト列として保存

def generate_frames(node):
    # フレームを生成するジェネレータ
    while True:
        if node.last_frame is not None:
            # フレームデータをHTTPレスポンスとして返すための形式に整形
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + node.last_frame + b'\r\n')

# ルートURL(192.168.***.***/)にアクセスされたときに実行される
@app.route('/')
def video():
    return Response(generate_frames(node), mimetype='multipart/x-mixed-replace; boundary=frame')

def main():
    rclpy.init(args=None)  # ROS 2初期化
    global node
    node = FlaskImageDisplay()  # ROS 2ノードのインスタンスを作成

    def ros_spin():
        rclpy.spin(node)

    # ROS 2ノードを別スレッドで実行
    thread = threading.Thread(target=ros_spin)
    thread.start()

    try:
        ip = ipget.ipget()  # IPアドレス取得ライブラリを使用
        ip_addr = ip.ipaddr("wlan0").split('/')[0]  # WLANインターフェースのIPアドレスを取得
        app.run(host=ip_addr, port=5000, debug=True)  # Flaskアプリケーションを開始
    finally:
        # アプリケーションが終了したら、ROS 2ノードを閉じてシャットダウン
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# 異なるトピックからの画像データを取得したい場合は、下記コマンドのように実行
# $ ros2 run smabo_pkg flask_image_display --remap /image:=＜新しいトピック名＞