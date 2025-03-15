import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SendSpeed(Node):
    def __init__(self):
        super().__init__('send_speed')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.send_velocity_command) 
        self.linear_speed = 0.5
        self.angular_speed = 0.0

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)
