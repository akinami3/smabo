import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2

class CamStream(Node):
    def __init__(self):
        super().__init__('cam_stream')

        # カメラを取得
        self.camera = cv2.VideoCapture(0)  # カメラデバイスを開く
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera.set(cv2.CAP_PROP_FPS, 30)

        # publisher定義
        self.publisher_ = self.create_publisher(Image, '/image', qos_profile_sensor_data) # センサーデータに適切なqos_profileを設定

        # OpenCVとROSイメージ間の変換を行うためのCvBridge
        self.bridge = CvBridge()

        # タイマーの設定。約0.033秒（30フレーム/秒）ごとにtimer_callback関数を呼び出します。
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        # カメラからフレームを取得
        ret, frame = self.camera.read()
        if ret:
            # OpenCVの画像をROSメッセージに変換
            ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg() # 現在時刻の情報を格納
            # 画像をパブリッシュ
            self.publisher_.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    cam_stream = CamStream()
    rclpy.spin(cam_stream)
    cam_stream.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
