import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

POINTS = {
    'goal    (-0.45, -0.49)': (-0.45, -0.49, 0.5),
    'robot   (-1.91, -0.82)': (-1.91, -0.82, 0.3),
    'hydrant ( 0.55, -0.49)': (0.55, -0.49, 0.3),
}


class CostQuery(Node):
    def __init__(self):
        super().__init__('cost_query')
        self.done = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_map, qos)

    def on_map(self, m):
        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        print(f'global costmap: {w}x{h} cells, {res} m/cell, origin ({ox:.2f}, {oy:.2f})')
        for name, (x, y, r) in POINTS.items():
            ci, cj = int((x - ox) / res), int((y - oy) / res)
            if not (0 <= ci < w and 0 <= cj < h):
                print(f'{name}: outside the costmap')
                continue
            n = int(r / res)
            free = blocked = unknown = 0
            for dj in range(-n, n + 1):
                for di in range(-n, n + 1):
                    if di * di + dj * dj > n * n:
                        continue
                    i, j = ci + di, cj + dj
                    if not (0 <= i < w and 0 <= j < h):
                        continue
                    v = m.data[j * w + i]
                    if v < 0:
                        unknown += 1
                    elif v >= 99:
                        blocked += 1
                    else:
                        free += 1
            print(f'{name}: cell cost {m.data[cj * w + ci]}; within {r} m -> '
                  f'passable {free}, blocked {blocked}, unknown {unknown}')
        self.done = True


rclpy.init()
node = CostQuery()
for _ in range(20):
    if node.done:
        break
    rclpy.spin_once(node, timeout_sec=1.0)
if not node.done:
    print('no costmap received in 20 s (is Nav2 running and active?)')
node.destroy_node()
rclpy.try_shutdown()
