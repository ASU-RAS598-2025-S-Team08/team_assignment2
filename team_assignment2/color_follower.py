import cv2 as cv
import rclpy as r
import numpy as np
import cv_bridge as c
import message_filters as m
from rclpy.node import Node
from sensor_msgs.msg import Image

class ColorFollower(Node):
    def __init__(self):
        super(ColorFollower, self).__init__(node_name='color_follower')
        self.declare_parameter('camera_stream', '/color/preview/image')
        self.r_hmin, self.r_smin, self.r_vmin = 0, 0, 0
        self.r_hmax, self.r_smax, self.r_vmax = 161, 255, 255
        self.g_hmin, self.g_smin, self.g_vmin = 45, 60, 0
        self.g_hmax, self.g_smax, self.g_vmax = 71, 255, 255
        self.b_hmin, self.b_smin, self.b_vmin = 98, 113, 0
        self.b_hmax, self.b_smax, self.b_vmax = 108, 255, 255
        self.convertor = c.CvBridge()
        self.get_logger().info(f"DETECTING FOLLOWING COLOR ----> HMIN={self.g_hmin} SMIN={self.g_smin} VMIN={self.g_vmin} HMAX={self.g_hmax} SMAX={self.g_smax} VMAX={self.g_vmax}")
        image_subscriber = m.Subscriber(self, Image, self.get_parameter('camera_stream').get_parameter_value().string_value)
        depth_subscriber = m.Subscriber(self, Image, '/stereo/depth')
        mixer = m.ApproximateTimeSynchronizer([image_subscriber, depth_subscriber], 10, 1)
        mixer.registerCallback(self.callback)

    def callback(self, image_message: Image, depth_message: Image):
        image = self.convertor.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
        image_h, image_w, _ = image.shape
        image_area = image_w * image_h
        depth_map = self.convertor.imgmsg_to_cv2(depth_message, desired_encoding='16UC1')
        depth_map = cv.resize(depth_map, (image_w, image_h))
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        r_lower = np.array([self.r_hmin, self.r_smin, self.r_vmin])
        r_upper = np.array([self.r_hmax, self.r_smax, self.r_vmax])
        r_mask = cv.inRange(hsv, r_lower, r_upper)
        g_lower = np.array([self.g_hmin, self.g_smin, self.g_vmin])
        g_upper = np.array([self.g_hmax, self.g_smax, self.g_vmax])
        g_mask = cv.inRange(hsv, g_lower, g_upper)
        b_lower = np.array([self.b_hmin, self.b_smin, self.b_vmin])
        b_upper = np.array([self.b_hmax, self.b_smax, self.b_vmax])
        b_mask = cv.inRange(hsv, b_lower, b_upper)
        mask = cv.bitwise_or(r_mask, g_mask)
        mask = cv.bitwise_or(mask, b_mask)
        contours, _ = cv.findContours(r_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        for index, contour in enumerate(contours):
            if cv.contourArea(contour) > 0.1 * image_area:
                xmin, ymin, w, h = cv.boundingRect(contour)
                x, y = int(xmin + (w / 2.0)), int(ymin + (h / 2.0))
                z = depth_map[y, x]
                image = cv.drawContours(image, contours, index, (255, 0, 255), 2)
                # image = cv.circle(image, (x, y), 2, (255, 0, 255), -1)
                image = cv.putText(image, f'Distance: {z}', (x - 10, y - 10), cv.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)
                break
        cv.imshow('Mask', mask)
        cv.imshow('Image', image)
        cv.waitKey(1)

def main(args=None):
    r.init()
    color_follower = ColorFollower()
    r.spin(color_follower)
    color_follower.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
