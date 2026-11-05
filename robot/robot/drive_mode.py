import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Polygon, Point32
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster


def quat_mul(a, b):
    """Quaternions product (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


class DriveModeNode(Node):
    def __init__(self):
        super().__init__('drive_mode')
        self.carrying = False

        # Footprint in frame base_link, as flattened list [x1, y1, x2, y2, ...].
        # Starting values: robot body + forklift. Changes when carrying.
        rect = [-0.20, 0.20, 0.475, 0.20, 0.475, -0.20, -0.20, -0.20]
        self.declare_parameter('footprint_empty', rect)
        self.declare_parameter('footprint_loaded', rect)

        self.tf_br = TransformBroadcaster(self)

        self.create_service(SetBool, 'set_carrying', self.on_set_carrying)

        # Subscribe to /cmd_vel to invert or not commands depending on mode
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_drive', 10)

        # ubscribe to /cmd_vel to rotate or not perceptions depending on mode
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_nav', 10)

        # Footprint towards both costmaps
        self.fp_pubs = [
            self.create_publisher(Polygon, '/local_costmap/footprint', 10),
            self.create_publisher(Polygon, '/global_costmap/footprint', 10),
        ]

        self.create_timer(0.02, self.publish_tf)        # 50 Hz
        self.create_timer(1.0, self.publish_footprint)  # ripubblicato a 1 Hz
        self.publish_footprint()

    # Mode changes
    def yaw(self):
        return math.pi if self.carrying else 0.0

    def on_set_carrying(self, req, res):
        self.carrying = req.data
        self.publish_footprint()
        res.success = True
        res.message = 'reverse' if self.carrying else 'forward'
        return res

    # TF base_link -> base_link_nav
    def publish_tf(self):
        y = self.yaw()
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'base_link_nav'
        t.transform.rotation.z = math.sin(y / 2.0)
        t.transform.rotation.w = math.cos(y / 2.0)
        self.tf_br.sendTransform(t)

    # Adjust speed commands wrt mode
    def on_cmd(self, msg):
        s = -1.0 if self.carrying else 1.0
        out = Twist()
        out.linear.x = s * msg.linear.x
        out.linear.y = s * msg.linear.y
        out.angular.z = msg.angular.z
        self.cmd_pub.publish(out)

    # Adjust odom perceptions wrt mode
    def on_odom(self, msg):
        s = -1.0 if self.carrying else 1.0
        y = self.yaw()
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = 'base_link_nav'
        out.pose = msg.pose

        # Rotate orientation quaternion wrt mode
        q = msg.pose.pose.orientation
        qr = quat_mul((q.x, q.y, q.z, q.w), (0.0, 0.0, math.sin(y / 2.0), math.cos(y / 2.0)))
        out.pose.pose.orientation.x = qr[0]
        out.pose.pose.orientation.y = qr[1]
        out.pose.pose.orientation.z = qr[2]
        out.pose.pose.orientation.w = qr[3]

        # Adjust speed commands wrt mode 
        out.twist = msg.twist
        out.twist.twist.linear.x = s * msg.twist.twist.linear.x
        out.twist.twist.linear.y = s * msg.twist.twist.linear.y
        out.twist.twist.angular.z = msg.twist.twist.angular.z
        self.odom_pub.publish(out)

    # Adjust published footprint wrt mode
    def publish_footprint(self):
        name = 'footprint_loaded' if self.carrying else 'footprint_empty'
        flat = self.get_parameter(name).value
        s = -1.0 if self.carrying else 1.0
        poly = Polygon()
        for i in range(0, len(flat), 2):
            poly.points.append(Point32(x=s * flat[i], y=s * flat[i + 1], z=0.0))
        for p in self.fp_pubs:
            p.publish(poly)


def main():
    rclpy.init()
    rclpy.spin(DriveModeNode())


if __name__ == '__main__':
    main()