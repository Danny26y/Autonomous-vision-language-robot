import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped support with tf2)


class TargetLocator(Node):
    def __init__(self):
        super().__init__('target_locator')
        self.declare_parameter('target_frame', 'map')
        self.target_frame = self.get_parameter('target_frame').value

        self.last_label = 'target'
        self.marker_id = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            String, '/perception/target_label', self.on_label, 10)
        self.create_subscription(
            PointStamped, '/perception/target_camera', self.on_point, 10)

        self.point_pub = self.create_publisher(
            PointStamped, '/perception/target_map', 10)
        self.marker_pub = self.create_publisher(
            Marker, '/perception/target_marker', 10)

        self.get_logger().info(
            f'target_locator up, transforming into "{self.target_frame}"')

    def on_label(self, msg):
        self.last_label = msg.data

    def on_point(self, msg):
        try:
            transformed = self.tf_buffer.transform(
                msg, self.target_frame, timeout=Duration(seconds=0.3))
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(
                f'could not transform target into "{self.target_frame}": {ex}',
                throttle_duration_sec=2.0)
            return

        self.point_pub.publish(transformed)
        self.publish_marker(transformed)

        self.get_logger().info(
            f'{self.last_label} @ {self.target_frame} = '
            f'({transformed.point.x:.2f}, {transformed.point.y:.2f}, '
            f'{transformed.point.z:.2f})',
            throttle_duration_sec=1.0)

    def publish_marker(self, point_stamped):
        m = Marker()
        m.header = point_stamped.header
        m.ns = 'targets'
        m.id = self.marker_id  # fixed id: this marker updates in place, it doesn't accumulate
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = point_stamped.point
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.1, 0.1, 0.9
        m.lifetime = Duration(seconds=2.0).to_msg()
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = TargetLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
