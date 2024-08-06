import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import threading
from flask import Flask, jsonify, request, Response
import os
import sys
import time
import pyrhexapi

app = Flask(__name__)
print("Server started")

# Global variables
cv_image = None
cv_bridge = CvBridge()
cv_image_lock = threading.Lock()  # Lock for thread-safe access to cv_image
custom_control_dir = None
custom_yolo_model_dir = None

def get_base_dir():
    current_file_path = os.path.abspath(__file__)
    return os.path.abspath(os.path.join(current_file_path, '..', '..', '..','..'))

BASE_DIR = get_base_dir()

@app.route('/')
def home():
    return 'Welcome to the Flask Server!\n'

@app.route('/image', methods=['GET'])
def get_image():
    global cv_image
    with cv_image_lock:  # Acquire lock before accessing cv_image
        if cv_image is None:
            return "No image available", 404
        ret, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 50])  # 50 is the quality level
        if not ret:
            return "Failed to encode image", 500
        return Response(jpeg.tobytes(), mimetype='image/jpeg')

@app.route('/control/<command>', methods=['POST'])
def control_robot(command):
    global rhex  # Access the global rhex object
    if command == "connect":
        mm = pyrhexapi.MultimediaParameters_t()
        mm.front_camera_receiver_port = 5500
        mm.rear_camera_receiver_port = 5501
        mm.robot_microphone_receiver_port = 5502
        intf = "lo"
        hostname = "127.0.0.1"
        port = 5000
        rhex = pyrhexapi.RHexAPI(intf, mm)
        try:
            rhex.Connect(hostname, port, 5001)
            print("Connected Successfully!")
            return jsonify({"status": "success", "message": "Robot connected successfully."}), 200
        except RuntimeError as e:
            print(f"Robot connection error: {e}")
            return jsonify({"status": "error", "message": str(e)}), 500
    elif command == "stand":
        if rhex is None:
            return jsonify({"status": "error", "message": "Robot not connected."}), 400
        try:
            rhex.SetMode(pyrhexapi.RHexMode.STAND)
            while rhex.GetMode() != pyrhexapi.RHexMode.STAND:
                time.sleep(0.1)
            return jsonify({"status": "success", "message": "Robot is now in STAND mode."}), 200
        except RuntimeError as e:
            return jsonify({"status": "error", "message": str(e)}), 500
    elif command == "sit":
        if rhex is None:
            return jsonify({"status": "error", "message": "Robot not connected."}), 400
        try:
            rhex.SetMode(pyrhexapi.RHexMode.SIT)
            while rhex.GetMode() != pyrhexapi.RHexMode.SIT:
                time.sleep(0.1)
            return jsonify({"status": "success", "message": "Robot is now in SIT mode."}), 200
        except RuntimeError as e:
            return jsonify({"status": "error", "message": str(e)}), 500
    elif command == "disconnect":
        if rhex is None:
            return jsonify({"status": "error", "message": "Robot not connected."}), 400
        print("Disconnecting from the robot...")
        try:
            rhex.SetMode(pyrhexapi.RHexMode.SIT)
            while rhex.GetMode() != pyrhexapi.RHexMode.SIT:
                time.sleep(0.1)
            rhex.Disconnect()
            rhex = None  # Reset the rhex object
            return jsonify({"status": "success", "message": "Robot disconnected."}), 200
        except RuntimeError as e:
            return jsonify({"status": "error", "message": str(e)}), 500
    elif command == "walk":
        if rhex is None:
            return jsonify({"status": "error", "message": "Robot not connected."}), 400
        try:
            rhex.SetMode(pyrhexapi.RHexMode.WALK)
            while rhex.GetMode() != pyrhexapi.RHexMode.WALK:
                time.sleep(0.1)
            return jsonify({"status": "success", "message": "Robot is now in WALK mode."}), 200
        except RuntimeError as e:
            return jsonify({"status": "error", "message": str(e)}), 500
    elif command == "forward":
        if rhex is None:
            return jsonify({"status": "error", "message": "Robot not connected."}), 400
        wc = pyrhexapi.WalkCommand_t()
        wc.type = pyrhexapi.WalkType.EFFICIENT_WALK
        wc.incline = 0
        wc.turning_speed = 0.0
        wc.direction = pyrhexapi.MovementDirection.FORWARD
        try:
            rhex.SetWalkCommand(wc)
            return jsonify({"status": "success", "message": "Robot moving forward."}), 200
        except RuntimeError as e:
            return jsonify({"status": "error", "message": str(e)}), 500
    elif command == "backward":
        if rhex is None:
            return jsonify({"status": "error", "message": "Robot not connected."}), 400
        wc = pyrhexapi.WalkCommand_t()
        wc.type = pyrhexapi.WalkType.EFFICIENT_WALK
        wc.incline = 0
        wc.turning_speed = 0.0
        wc.direction = pyrhexapi.MovementDirection.BACKWARD
        try:
            rhex.SetWalkCommand(wc)
            return jsonify({"status": "success", "message": "Robot moving backward."}), 200
        except RuntimeError as e:
            return jsonify({"status": "error", "message": str(e)}), 500
    elif command == "stop":
        if rhex is None:
            return jsonify({"status": "error", "message": "Robot not connected."}), 400
        wc = pyrhexapi.WalkCommand_t()
        wc.type = pyrhexapi.WalkType.EFFICIENT_WALK
        wc.incline = 0
        wc.turning_speed = 0.0
        wc.direction = pyrhexapi.MovementDirection.STOP
        try:
            rhex.SetWalkCommand(wc)
            return jsonify({"status": "success", "message": "Robot stopped."}), 200
        except RuntimeError as e:
            return jsonify({"status": "error", "message": str(e)}), 500
    elif command == "right":
        if rhex is None:
            return jsonify({"status": "error", "message": "Robot not connected."}), 400
        wc = pyrhexapi.WalkCommand_t()
        wc.type = pyrhexapi.WalkType.EFFICIENT_WALK
        wc.incline = 0
        wc.turning_speed = 0.15
        wc.direction = pyrhexapi.MovementDirection.STOP
        try:
            rhex.SetWalkCommand(wc)
            return jsonify({"status": "success", "message": "Robot turning right."}), 200
        except RuntimeError as e:
            return jsonify({"status": "error", "message": str(e)}), 500
    elif command == "left":
        if rhex is None:
            return jsonify({"status": "error", "message": "Robot not connected."}), 400
        wc = pyrhexapi.WalkCommand_t()
        wc.type = pyrhexapi.WalkType.EFFICIENT_WALK
        wc.incline = 0
        wc.turning_speed = -0.15
        wc.direction = pyrhexapi.MovementDirection.STOP
        try:
            rhex.SetWalkCommand(wc)
            return jsonify({"status": "success", "message": "Robot turning left."}), 200
        except RuntimeError as e:
            return jsonify({"status": "error", "message": str(e)}), 500
    elif command == "calibrate":
        if rhex is None:
            return jsonify({"status": "error", "message": "Robot not connected."}), 400
        try:
            while True:
                rhex.SetMode(pyrhexapi.RHexMode.SIT)
                while rhex.GetMode() != pyrhexapi.RHexMode.SIT:
                    time.sleep(0.1)
                time.sleep(1)

                rhex.SetMode(pyrhexapi.RHexMode.CALIBRATION)
                while rhex.GetMode() != pyrhexapi.RHexMode.CALIBRATION:
                    time.sleep(0.1)
                cc = pyrhexapi.CalibrationCommand_t()
                cc.type = pyrhexapi.CalibrationType.GROUND
                rhex.SetCalibrationCommand(cc)
                time.sleep(1)
                
                calibrated = False
                retries = 0

                while not calibrated:
                    retries += 1
                    cs = rhex.GetCalibrationState()
                    calibrated = all(status == pyrhexapi.CalibrationStatus.CALIBRATED for status in cs.leg_status)
                    if retries > 300:
                        return jsonify({"status": "error", "message": "Calibration timed out."}), 500
                    time.sleep(0.01)

                if calibrated:
                    return jsonify({"status": "success", "message": "Calibration successful."}), 200
        except RuntimeError as e:
            return jsonify({"status": "error", "message": f"Exception caught during calibration: {e}"}), 500
    else:
        return jsonify({"status": "error", "message": "Invalid command."}), 400

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')

        self.image_subscription = self.create_subscription(
            Image,
            '/depth_camera/image_raw',
            self.image_callback,
            10
        )

        self.cv_bridge = CvBridge()
        model_path = custom_yolo_model_dir if custom_yolo_model_dir else os.path.join(BASE_DIR, 'src', 'rhex_yolo_detector', 'rhex_yolo_detector', 'best.pt')
        self.detector = YOLO(model_path)
        self.current_position = None

    def image_callback(self, msg):
        global cv_image
        try:
            image_bgr = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            #image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
            results = self.detector(image_bgr)

            objects_detected = False
            if results:
                for result in results:
                    if result.boxes:
                        objects_detected = True
                        for box in result.boxes:
                            x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                            cv2.rectangle(image_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        break

            # Update global cv_image with lock
            with cv_image_lock:
                cv_image = image_bgr

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def position_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_position = (position, orientation)

def ros2_setup():
    rclpy.init()
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    if '-cp' in sys.argv:
        custom_yolo_model_dir = input("Enter the absolute path of the YOLO model directory: ")
    
    ros2_thread = threading.Thread(target=ros2_setup)
    ros2_thread.start()
    app.run(host='0.0.0.0', port=5001)
