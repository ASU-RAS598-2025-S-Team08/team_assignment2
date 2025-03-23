import rclpy as r
from rclpy.node import Node
from geometry_msgs.msg import Twist
from team_assignment2_msgs.msg import Blob

class RedFollower(Node):
    def __init__(self):
        super(RedFollower, self).__init__(node_name='red_follower')
        self.cmd_publisher = self.create_publisher(Twist, '/c3_12/cmd_vel')
        blob_subscriber = self.create_subscription(Blob, '/blob', self.callback)

    def callback(self, blob_message: Blob):
        r_blobs = blob_message.objects[0]
        r_areas = blob_message.areas[0:r_blobs]
        index = r_areas.index(max(r_areas))
        r_x = blob_message.xs[0:r_blobs][index]
        r_y = blob_message.ys[0:r_blobs][index]
        r_z = blob_message.zs[0:r_blobs][index]
        if r_x > blob_message.width / 2.0 or r_x < blob_message.width / 2.0:
            pass

def main(args=None):
    r.init(args=args)
    red_follower = RedFollower()
    r.spin(red_follower)
    red_follower.destroy_node()
    r.shutdown()

if __name__ == '__main__':
    main()
