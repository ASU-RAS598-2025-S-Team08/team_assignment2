import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SendSpeed2(Node):
    def __init__(self):
        super().__init__('send_speed2')
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
       
        self.subscription = self.create_subscription(
            Float32MultiArray,  # Assuming the node contain [x, y] position of the blob
            'red_blob_position',
            self.blob_position_callback,
            10
        )

        self.linear_speed = 0.1
        self.angular_speed = 0.0

    def blob_position_callback(self, msg):

        x = msg.data[0]  # x-coordinate of the blob
        y = msg.data[1]  # y-coordinate of the blob

        # Assume that the camera image has a resolution of 640x480
        image_center_x = 320  
        image_center_y = 240  

        # Calculate how far the blob is from the center of the image
        error_x = x - image_center_x 
        error_y = y - image_center_y  

        # Proportional control for angular velocity based on horizontal offset
        angular_speed = 0.005 * error_x 
        
        # Proportional control for linear velocity based on vertical offset
        linear_speed = 0.5 * (abs(error_y) / image_center_y)  

        # Set the linear and angular speeds to the calculated values
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        vel_msg = Twist()
        vel_msg.linear.x = self.linear_speed
        vel_msg.angular.z = self.angular_speed

        # Publish the velocity message to cmd_vel
        self.publisher_.publish(vel_msg) 
        self.get_logger().info(f"Publishing: Linear Speed: {self.linear_speed}, Angular Speed: {self.angular_speed}")

