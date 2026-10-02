import math
from collections import deque

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.declare_parameter('standoff', 1.0)
        self.declare_parameter('min_detections', 5)
        self.declare_parameter('max_retries', 2)
        self.standoff = float(self.get_parameter('standoff').value)
        self.min_det = int(self.get_parameter('min_detections').value)
        self.max_retries = int(self.get_parameter('max_retries').value)

        self.state = 'SEARCH'
        self.points = deque(maxlen=self.min_det)
        self.retries = 0
        self.target = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(
            PointStamped, '/perception/target_map', self.on_target, 10)
        self.get_logger().info('mission_controller up: state SEARCH')

    def on_target(self, msg):
        if self.state != 'SEARCH':
            return
        self.points.append((msg.point.x, msg.point.y))
        if len(self.points) < self.min_det:
            return
        xs = sorted(p[0] for p in self.points)
        ys = sorted(p[1] for p in self.points)
        self.target = (xs[len(xs) // 2], ys[len(ys) // 2])
        self.get_logger().info(
            f'target confirmed at map ({self.target[0]:.2f}, {self.target[1]:.2f})')
        self.state = 'APPROACH'
        self.send_goal()

    def send_goal(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'no robot pose yet: {ex}')
            self.back_to_search()
            return
        if not self.client.server_is_ready():
            self.get_logger().warn('Nav2 action server not ready, will retry on the next detection')
            self.back_to_search()
            return

        rx, ry = t.transform.translation.x, t.transform.translation.y
        tx, ty = self.target
        dx, dy = tx - rx, ty - ry
        dist = math.hypot(dx, dy)
        if dist <= self.standoff:
            self.get_logger().info('already within the stand-off distance: mission complete')
            self.state = 'DONE'
            return

        gx = tx - dx / dist * self.standoff
        gy = ty - dy / dist * self.standoff
        yaw = math.atan2(dy, dx)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'driving to ({gx:.2f}, {gy:.2f}), {self.standoff} m short of the target')
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_feedback(self, fb):
        self.get_logger().info(
            f'distance remaining: {fb.feedback.distance_remaining:.2f} m',
            throttle_duration_sec=2.0)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('goal rejected by Nav2')
            self.on_failed()
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('arrived: mission complete')
            self.state = 'DONE'
        else:
            self.get_logger().warn(f'navigation ended with status {status}')
            self.on_failed()

    def on_failed(self):
        self.retries += 1
        if self.retries > self.max_retries:
            self.get_logger().error('giving up after repeated failures')
            self.state = 'DONE'
            return
        self.get_logger().info(f'retry {self.retries}/{self.max_retries}: back to SEARCH')
        self.back_to_search()

    def back_to_search(self):
        self.state = 'SEARCH'
        self.points.clear()


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
