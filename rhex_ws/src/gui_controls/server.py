import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
from datetime import datetime
import threading
from flask import Flask, jsonify, request, Response
import subprocess
import os
import socket
import sys

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
    return os.path.abspath(os.path.join(current_file_path, '..', '..', '..'))

BASE_DIR = get_base_dir()

def run_control_client(command):
    try:
        run_dir = custom_control_dir if custom_control_dir else os.path.join(BASE_DIR, 'src', 'controls')
        if not os.path.exists(run_dir):
            return jsonify({'error': f'Directory not found: {run_dir}'}), 500
        os.chdir(run_dir)
        dir_contents = os.listdir('.')
        print(f"Current directory contents: {dir_contents}")
        run_command = ['./control_client', command]
        run_result = subprocess.run(run_command, capture_output=True, text=True)
        return jsonify({
            'run_output': run_result.stdout,
            'run_errors': run_result.stderr,
            'returncode': run_result.returncode
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/')
def home():
    return 'Welcome to the Flask Server!\n'

@app.route('/run/<command>', methods=['POST'])
def run_command(command):
    return run_control_client(command)

@app.route('/image', methods=['GET'])
def get_image():
    global cv_image
    with cv_image_lock:  # Acquire lock before accessing cv_image
        if cv_image is None:
            return "No image available", 404
        ret, jpeg = cv2.imencode('.jpg', cv_image)
        if not ret:
            return "Failed to encode image", 500
        return Response(jpeg.tobytes(), mimetype='image/jpeg')

@app.route('/control/<command>', methods=['POST'])
def control_robot(command):
    try:
        server_address = '127.0.0.1'
        port = 8080
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((server_address, port))
            sock.sendall(command.upper().encode())
            response = sock.recv(1024).decode()
            return jsonify({'response': response})
    except Exception as e:
        return jsonify({'error': str(e)}), 500

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
            cv_image_local = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            results = self.detector(cv_image_local)

            objects_detected = False
            if results:
                for result in results:
                    if result.boxes:
                        objects_detected = True
                        for box in result.boxes:
                            x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                            cv2.rectangle(cv_image_local, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        break

            # Update global cv_image with lock
            with cv_image_lock:
                cv_image = cv_image_local

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
        custom_control_dir = input("Enter the absolute path of the controls directory: ")
        custom_yolo_model_dir = input("Enter the absolute path of the YOLO model directory: ")

    ros2_thread = threading.Thread(target=ros2_setup)
    ros2_thread.start()
    app.run(host='0.0.0.0', port=5001)
