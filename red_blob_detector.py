import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedBlobTracker(Node):
    def __init__(self):
        super().__init__('red_blob_tracker')

        # Subscribe to Oak-D camera topic
        self.subscription = self.create_subscription(
            Image, '/oakd/rgb/image_raw', self.process_image, 10)

        # Publisher for TurtleBot movement commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        self.image_width = 640  # Camera resolution width
        self.center_tolerance = 50  # Pixel range where the blob is considered centered
        self.max_speed = 0.2  # Max forward speed
        self.max_turn = 0.5  # Max turn speed

    def process_image(self, msg):
        """Processes camera feed, detects red blobs, and moves TurtleBot towards the largest red blob."""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define red color range (covers both shades of red)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Create a mask to detect red objects
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Find contours of detected red regions
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest red blob
            largest_blob = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_blob)
            blob_center_x = x + w // 2
            blob_size = w * h  # Blob area
            
            # Normalize blob size to a range [0, 1] for speed scaling
            normalized_size = min(blob_size / (self.image_width * 480), 1.0)

            # Debugging - print detected blob information
            self.get_logger().info(f"Largest red blob at ({blob_center_x}, {y}) size: {blob_size}")

            # Generate movement commands
            twist_msg = Twist()

            # Adjust angular speed based on horizontal position
            if blob_center_x < self.image_width // 2 - self.center_tolerance:
                twist_msg.angular.z = self.max_turn  # Turn left
            elif blob_center_x > self.image_width // 2 + self.center_tolerance:
                twist_msg.angular.z = -self.max_turn  # Turn right
            else:
                twist_msg.angular.z = 0.0  # Stop turning

            # Adjust forward speed based on blob size
            twist_msg.linear.x = self.max_speed * (1 - normalized_size)

            # Publish velocity command
            self.publisher.publish(twist_msg)

            # Draw rectangle around the detected blob
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Show the processed image for debugging
        cv2.imshow("Red Blob Detection", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = RedBlobTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
