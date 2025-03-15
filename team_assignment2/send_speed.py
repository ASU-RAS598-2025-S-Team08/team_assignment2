import rclpy as r
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SpeedController(Node):
    def __init__(self):
        super(SpeedController, self).__init__(node_name='speed_controller')
