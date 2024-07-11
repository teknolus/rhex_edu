import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
from datetime import datetime

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        
        # Image subscription
        self.image_subscription = self.create_subscription(
            Image,
            '/depth_camera/image_raw',
            self.image_callback,
            10
        )
        
        # Odometry subscription (position)
        self.position_subscription = self.create_subscription(
            Odometry,
            '/odom/robot_pos',
            self.position_callback,
            10
        )
        
        
        self.cv_bridge = CvBridge()
        self.detector = YOLO('/home/rhex/mnt/rhex_ws/src/rhex_yolo_detector/rhex_yolo_detector/best.pt')
        self.current_position = None

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            # Perform object detection using YOLO
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            results = self.detector(cv_image)
            
            # Check if any objects were detected
            objects_detected = False
            if results:
                for result in results:
                    if result.boxes:  # Check if there are any bounding boxes
                        objects_detected = True
                        # Draw bounding boxes on the image
                        for box in result.boxes:
                            x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                        break
            
            if objects_detected:
                position, orientation = self.current_position
                self.get_logger().info(f'Objects detected in the image. Position: x={position.x}, y={position.y}, z={position.z}')
                self.get_logger().info(f'Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}')
                detection_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')  # Get the current time and format it
                self.get_logger().info(f'Objects detected in the image at {detection_time}')
            else:
                self.get_logger().info('No Humans Detected')

            cv2.imshow('Depth Image', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def position_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_position = (position, orientation)

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
