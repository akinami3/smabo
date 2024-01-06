from flask import Flask, Response
import cv2
import ipget

app = Flask(__name__)

def generate_frames():
    camera = cv2.VideoCapture(0)  # カメラデバイスを開く
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 
    camera.set(cv2.CAP_PROP_FPS, 30)

    while True:
        success, frame = camera.read()  # フレームを取得
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    ip = ipget.ipget()
    ip_addr = ip.ipaddr("wlan0").split('/')[0] # wifiのip addressを取得
    app.run(host=ip_addr, port=5000, debug=True)