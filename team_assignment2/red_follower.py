import rclpy as r
from rclpy.node import Node
from geometry_msgs.msg import Twist
from team_assignment2_msgs.msg import Blob

class RedFollower(Node):
    def __init__(self):
        super(RedFollower, self).__init__(node_name='red_follower')
        self.closest = 1
        self.angular_speed = 0.005
        self.linear_speed = 0.002
        self.cmd_publisher = self.create_publisher(Twist, '/c3_12/cmd_vel', 10)
        blob_subscriber = self.create_subscription(Blob, '/blob', self.callback, 10)

    def callback(self, blob_message: Blob):
        r_blobs = blob_message.objects[0]
        r_areas = blob_message.areas[0:r_blobs]
        cmd_message = Twist()
        if len(r_areas) == 0:
            cmd_message.angular.z = -1 * self.angular_speed
            self.cmd_publisher.publish(cmd_message)
            self.get_logger().info("COULDN'T FIND RED....")
        else:
            index = r_areas.index(max(r_areas))
            r_x = blob_message.xs[0:r_blobs][index]
            r_y = blob_message.ys[0:r_blobs][index]
            r_z = blob_message.zs[0:r_blobs][index]
            self.get_logger().info(f"FOUND RED AT: {r_x} {r_y} {r_z}")
            if r_z > self.closest:
                cmd_message.linear.z = self.linear_speed
                self.cmd_publisher.publish(cmd_message)
            else:
                cmd_message.linear.z = -1 * self.linear_speed
                self.cmd_publisher.publish(cmd_message)

def main(args=None):
    r.init(args=args)
    red_follower = RedFollower()
    r.spin(red_follower)
    red_follower.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
