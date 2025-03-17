import cv2 as cv
import rclpy as r
import cv_bridge as c
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraViewer(Node):
    def __init__(self):
        super(CameraViewer, self).__init__(node_name='camera_viewer')
        self.declare_parameter('camera_stream', '/color/preview/image')
        self.convertor = c.CvBridge()
        self.create_subscription(Image, self.get_parameter('camera_stream').get_parameter_value().string_value, self.callback, 10)

    def callback(self, image_message: Image):
        image = self.convertor.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
        self.get_logger().info(f'RECEIVED IMAGE OF SIZE {image.shape} AND RANGE [{image.min()}, {image.max()}]')
        cv.imshow('Image', image)
        cv.waitKey(1)

def main(args=None):
    r.init(args=args)
    camera_viewer = CameraViewer()
    r.spin(camera_viewer)
    camera_viewer.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
