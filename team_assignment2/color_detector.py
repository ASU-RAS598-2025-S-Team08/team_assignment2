import cv2 as cv
import rclpy as r
import numpy as np
import cv_bridge as c
import message_filters as m
from rclpy.node import Node
from sensor_msgs.msg import Image
from team_assignment2_msgs.msg import Blob

class ColorDetector(Node):
    def __init__(self):
        super(ColorDetector, self).__init__(node_name='color_detector')
        self.declare_parameter('camera_stream', '/color/preview/image')
        self.r_hmin, self.r_smin, self.r_vmin = 163, 80, 119
        self.r_hmax, self.r_smax, self.r_vmax = 179, 214, 255
        self.get_logger().info(f"RED ----> HMIN={self.r_hmin} SMIN={self.r_smin} VMIN={self.r_vmin} HMAX={self.r_hmax} SMAX={self.r_smax} VMAX={self.r_vmax}")
        self.g_hmin, self.g_smin, self.g_vmin = 45, 60, 0
        self.g_hmax, self.g_smax, self.g_vmax = 71, 255, 255
        self.get_logger().info(f"GREEN ----> HMIN={self.g_hmin} SMIN={self.g_smin} VMIN={self.g_vmin} HMAX={self.g_hmax} SMAX={self.g_smax} VMAX={self.g_vmax}")
        self.b_hmin, self.b_smin, self.b_vmin = 98, 113, 0
        self.b_hmax, self.b_smax, self.b_vmax = 108, 255, 255
        self.get_logger().info(f"BLUE ----> HMIN={self.b_hmin} SMIN={self.b_smin} VMIN={self.b_vmin} HMAX={self.b_hmax} SMAX={self.b_smax} VMAX={self.b_vmax}")
        self.convertor = c.CvBridge()
        self.blob_publisher = self.create_publisher(Blob, '/blob', 10)
        image_subscriber = m.Subscriber(self, Image, self.get_parameter('camera_stream').get_parameter_value().string_value)
        depth_subscriber = m.Subscriber(self, Image, '/stereo/depth')
        synchronizer = m.ApproximateTimeSynchronizer([image_subscriber, depth_subscriber], 10, 1)
        synchronizer.registerCallback(self.callback)

    def callback(self, image_message: Image, depth_message: Image):
        image = self.convertor.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
        image_h, image_w, _ = image.shape
        depth_map = self.convertor.imgmsg_to_cv2(depth_message, desired_encoding='16UC1')
        depth_map = cv.resize(depth_map, (image_w, image_h))
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        ##########################################################################################
        r_lower = np.array([self.r_hmin, self.r_smin, self.r_vmin])
        r_upper = np.array([self.r_hmax, self.r_smax, self.r_vmax])
        r_mask = cv.inRange(hsv, r_lower, r_upper)
        r_mask = cv.erode(r_mask, kernel=np.ones(shape=(5, 5)), iterations=1)
        r_mask = cv.dilate(r_mask, kernel=np.ones(shape=(3, 3)), iterations=2)
        r_contours, _ = cv.findContours(r_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        r_areas = []
        r_xs = []
        r_ys = []
        r_zs = []
        for r_contour in r_contours:
            r_area = cv.contourArea(r_contour)
            r_areas.append(r_area)
            r_moment = cv.moments(r_contour)
            r_x = int(r_moment["m10"] / r_moment["m00"])
            r_xs.append(r_x)
            r_y = int(r_moment["m01"] / r_moment["m00"])
            r_ys.append(r_y)
            r_z = int(depth_map[r_y, r_x])
            r_zs.append(r_z)
        image = cv.drawContours(image, r_contours, -1, (0, 255, 0), 2)
        # index = r_areas.index(max(r_areas))
        # r_x, r_y, r_z = r_xs[index], r_ys[index], depth_map[r_y, r_x]
        # self.get_logger().info(f"{r_x}, {r_y}, {r_z}")
        # image = cv.circle(image, (r_x, r_y), 2, (0, 255, 0), -1)
        ##########################################################################################
        g_lower = np.array([self.g_hmin, self.g_smin, self.g_vmin])
        g_upper = np.array([self.g_hmax, self.g_smax, self.g_vmax])
        g_mask = cv.inRange(hsv, g_lower, g_upper)
        g_mask = cv.erode(g_mask, kernel=np.ones(shape=(5, 5)), iterations=1)
        g_mask = cv.dilate(g_mask, kernel=np.ones(shape=(3, 3)), iterations=2)
        g_contours, _ = cv.findContours(g_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        g_areas = []
        g_xs = []
        g_ys = []
        g_zs = []
        for g_contour in g_contours:
            g_area = cv.contourArea(g_contour)
            g_areas.append(g_area)
            g_moment = cv.moments(g_contour)
            g_x = int(g_moment["m10"] / g_moment["m00"])
            g_xs.append(g_x)
            g_y = int(g_moment["m01"] / g_moment["m00"])
            g_ys.append(g_y)
            g_z = int(depth_map[g_y, g_x])
            g_zs.append(g_z)
        # image = cv.drawContours(image, g_contours, -1, (255, 0, 0), 2)
        ##########################################################################################
        b_lower = np.array([self.b_hmin, self.b_smin, self.b_vmin])
        b_upper = np.array([self.b_hmax, self.b_smax, self.b_vmax])
        b_mask = cv.inRange(hsv, b_lower, b_upper)
        b_mask = cv.erode(b_mask, kernel=np.ones(shape=(5, 5)), iterations=1)
        b_mask = cv.dilate(b_mask, kernel=np.ones(shape=(3, 3)), iterations=2)
        b_contours, _ = cv.findContours(b_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        b_areas = []
        b_xs = []
        b_ys = []
        b_zs = []
        for b_contour in b_contours:
            b_area = cv.contourArea(b_contour)
            b_areas.append(b_area)
            b_moment = cv.moments(b_contour)
            b_x = int(b_moment["m10"] / b_moment["m00"])
            b_xs.append(b_x)
            b_y = int(b_moment["m01"] / b_moment["m00"])
            b_ys.append(b_y)
            b_z = int(depth_map[b_y, b_x])
            b_zs.append(b_z)
        # image = cv.drawContours(image, b_contours, -1, (0, 0, 255), 2)
        ##########################################################################################
        cv.imshow('Image', image)
        cv.waitKey(1)
        ##########################################################################################
        blob_message = Blob()
        blob_message.objects = [len(b_areas), len(g_areas), len(b_areas)]
        blob_message.areas = r_areas + g_areas + b_areas
        blob_message.xs = r_xs + g_xs + b_xs
        blob_message.ys = r_ys + g_ys + b_ys
        blob_message.zs = r_zs + g_zs + b_zs
        blob_message.width = image_w
        blob_message.height = image_h
        self.blob_publisher.publish(blob_message)

def main(args=None):
    r.init()
    color_follower = ColorDetector()
    r.spin(color_follower)
    color_follower.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
