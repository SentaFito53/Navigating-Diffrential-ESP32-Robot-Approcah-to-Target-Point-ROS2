import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Twist, TransformStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from rclpy.duration import Duration
import math


class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_deg_ = 0.0
        self.target_x_ = 0.0
        self.target_y_ = 0.0

        self.pos_sub_ = self.create_subscription(
            Point,
            '/robot_pos',
            self.position_callback,
            10
        )

        self.heading_sub_ = self.create_subscription(
            Float32,
            '/heading',
            self.heading_callback,
            10
        )

        self.target_sub_ = self.create_subscription(
            Point,
            '/target_pos',
            self.target_callback,
            10
        )

        self.cmd_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub_ = self.create_publisher(Marker, '/target_marker', 10)
        self.odom_pub_ = self.create_publisher(Odometry, '/odom', 10)

        self.tf_broadcaster_ = TransformBroadcaster(self)

        self.timer_ = self.create_timer(0.05, self.publish_odometry_and_tf)

    def position_callback(self, msg):
        self.x_ = msg.x / 100.0 * -1
        self.y_ = msg.y / 100.0

    def heading_callback(self, msg):
        self.theta_deg_ = -msg.data

    def target_callback(self, msg):
        self.target_x_ = msg.x / 100.0 * -1
        self.target_y_ = msg.y / 100.0
        self.publish_target_marker(self.target_x_, self.target_y_)

    def publish_target_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=0).to_msg()

        self.marker_pub_.publish(marker)

    def publish_odometry_and_tf(self):
        now = self.get_clock().now().to_msg()
        theta_rad = self.theta_deg_ * math.pi / 180.0

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x_
        t.transform.translation.y = self.y_
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, theta_rad)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster_.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x_
        odom.pose.pose.position.y = self.y_
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.angular.z = 0.0

        self.odom_pub_.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

