import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SpeedController(Node):
    def __init__(self):
        super().__init__('speed_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.send_velocity_command) 
        self.linear_speed = 0.5
        self.angular_speed = 0.0

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    wheel_controller = send_velocity_command()
    rclpy.spin(wheel_controller)
    wheel_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
