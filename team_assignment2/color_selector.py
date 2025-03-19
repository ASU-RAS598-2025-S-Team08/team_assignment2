import cv2 as cv
import rclpy as r
import numpy as np
import cv_bridge as c
from rclpy.node import Node
from sensor_msgs.msg import Image

class ColorSelector(Node):
    def __init__(self):
        super(ColorSelector, self).__init__(node_name='color_detector')
        self.declare_parameter('camera_stream', '/color/preview/image')
        cv.namedWindow('TRACKBARS')
        cv.createTrackbar('HMIN', 'TRACKBARS', 0, 179, self.nothing)
        cv.createTrackbar('HMAX', 'TRACKBARS', 179, 179, self.nothing)
        cv.createTrackbar('SMIN', 'TRACKBARS', 0, 255, self.nothing)
        cv.createTrackbar('SMAX', 'TRACKBARS', 255, 255, self.nothing)
        cv.createTrackbar('VMIN', 'TRACKBARS', 0, 255, self.nothing)
        cv.createTrackbar('VMAX', 'TRACKBARS', 255, 255, self.nothing)
        self.convertor = c.CvBridge()
        image_subscriber = self.create_subscription(Image, self.get_parameter('camera_stream').get_parameter_value().string_value, self.callback, 10)

    def nothing(self, _):
        pass

    def callback(self, image_message: Image):
        img = self.convertor.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        hmin = cv.getTrackbarPos('HMIN', 'TRACKBARS')
        smin = cv.getTrackbarPos('SMIN', 'TRACKBARS')
        vmin = cv.getTrackbarPos('VMIN', 'TRACKBARS')
        hmax = cv.getTrackbarPos('HMAX', 'TRACKBARS')
        smax = cv.getTrackbarPos('SMAX', 'TRACKBARS')
        vmax = cv.getTrackbarPos('VMAX', 'TRACKBARS')
        lower = np.array([hmin, smin, vmin])
        upper = np.array([hmax, smax, vmax])
        mask = cv.inRange(hsv, lower, upper)
        cv.imshow('Mask', mask)
        cv.imshow('Image', img)
        cv.waitKey(1)

def main(args=None):
    r.init()
    color_selector = ColorSelector()
    r.spin(color_selector)
    color_selector.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
