import cv2 as cv
import rclpy as r
import numpy as np
import cv_bridge as c
from rclpy.node import Node
from sensor_msgs.msg import Image

class ColorDetector(Node):
    def __init__(self):
        super(ColorDetector, self).__init__(node_name='color_detector')
        self.declare_parameter('color', 'black')
        self.declare_parameter('camera_stream', '/color/preview/image')
        if self.get_parameter('color').get_parameter_value().string_value == 'black':
            self.hmin, self.smin, self.vmin, self.hmax, self.smax, self.vmax = 0, 0, 0, 179, 255, 255
        elif self.get_parameter('color').get_parameter_value().string_value == 'white':
            self.hmin, self.smin, self.vmin, self.hmax, self.smax, self.vmax = 0, 0, 0, 179, 255, 255
        elif self.get_parameter('color').get_parameter_value().string_value == 'red':
            self.hmin, self.smin, self.vmin, self.hmax, self.smax, self.vmax = 0, 0, 0, 179, 255, 255
        elif self.get_parameter('color').get_parameter_value().string_value == 'green':
            self.hmin, self.smin, self.vmin, self.hmax, self.smax, self.vmax = 0, 0, 0, 179, 255, 255
        elif self.get_parameter('color').get_parameter_value().string_value == 'blue':
            self.hmin, self.smin, self.vmin, self.hmax, self.smax, self.vmax = 0, 0, 0, 179, 255, 255
        elif self.get_parameter('color').get_parameter_value().string_value == 'yellow':
            self.hmin, self.smin, self.vmin, self.hmax, self.smax, self.vmax = 0, 0, 0, 179, 255, 255
        self.convertor = c.CvBridge()
        image_subscriber = self.create_subscription(Image, self.get_parameter('camera_stream').get_parameter_value().string_value, self.callback, 10)

    def callback(self, image_message: Image):
        img = self.convertor.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower = np.array([self.hmin, self.smin, self.vmin])
        upper = np.array([self.hmax, self.smax, self.vmax])
        mask = cv.inRange(hsv, lower, upper)
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv.contourArea(contour) > 500:  # Filter small noise
                x, y, w, h = cv.boundingRect(contour)
                cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw bounding box
        cv.imshow('Image', img)
        cv.waitKey(1)
