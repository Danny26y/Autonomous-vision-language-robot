import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


GOALS = [
    (-0.45, -0.49, 'near, known-good'),
    (0.0, 0.0, 'through the pillar field'),
    (1.0, -0.5, 'far side, near hydrant'),
]


class PlanTimer(Node):
    def __init__(self):
        super().__init__('plan_timer')
        self.client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    def time_one(self, x, y, label):
        if not self.client.wait_for_server(timeout_sec=5.0):
            print(f'{label}: server not available')
            return
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.w = 1.0
        goal.use_start = False

        t0 = time.perf_counter()
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            print(f'{label}: goal rejected')
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wall_ms = (time.perf_counter() - t0) * 1000.0
        result = result_future.result().result
        n_poses = len(result.path.poses)
        pt = result.planning_time
        planner_ms = pt.sec * 1000.0 + pt.nanosec / 1e6
        print(f'{label}: planner reported {planner_ms:.1f} ms '
              f'(wall-clock {wall_ms:.0f} ms), path has {n_poses} poses')


rclpy.init()
node = PlanTimer()
for x, y, label in GOALS:
    node.time_one(x, y, label)
node.destroy_node()
rclpy.shutdown()
