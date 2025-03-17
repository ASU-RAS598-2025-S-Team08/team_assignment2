import cv2 as cv
import rclpy as r
import cv_bridge as c
from rclpy.node import Node
from sensor_msgs.msg import Image

class DepthViewer(Node):
    def __init__(self):
        super(DepthViewer, self).__init__(node_name='depth_viewer')
        self.convertor = c.CvBridge()
        self.create_subscription(Image, '/stereo/depth', self.callback, 10)

    def callback(self, depth_message: Image):
        depth_map = self.convertor.imgmsg_to_cv2(depth_message, desired_encoding='16UC1')
        self.get_logger().info(f'RECEIVED IMAGE OF SIZE {depth_map.shape} AND RANGE [{depth_map.min()}, {depth_map.max()}]')
        cv.imshow('Depth Map', depth_map)
        cv.waitKey(1)

def main(args=None):
    r.init(args=args)
    depth_viewer = DepthViewer()
    r.spin(depth_viewer)
    depth_viewer.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
