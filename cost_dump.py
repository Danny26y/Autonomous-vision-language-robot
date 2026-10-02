import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

MARKS = {'R': (-1.08, -0.80), 'G': (-0.45, -0.49), 'H': (0.55, -0.49)}


class Dump(Node):
    def __init__(self):
        super().__init__('cost_dump')
        self.done = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_map, qos)

    def on_map(self, m):
        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        W, H = (w + 1) // 2, (h + 1) // 2            # 2x2 cells per character
        marks = {}
        for ch, (x, y) in MARKS.items():
            marks[(int((x - ox) / res) // 2, int((y - oy) / res) // 2)] = ch
        print(f'origin ({ox:.2f}, {oy:.2f}); each character = {2 * res:.1f} m; top row = +y')
        print('R robot, G goal, H hydrant | # blocked, + high cost, . inflated, blank free, ? unknown')
        for J in range(H - 1, -1, -1):
            row = []
            for I in range(W):
                if (I, J) in marks:
                    row.append(marks[(I, J)])
                    continue
                vals = [m.data[j * w + i]
                        for j in (2 * J, 2 * J + 1) for i in (2 * I, 2 * I + 1)
                        if j < h and i < w]
                v = max(vals)                          # keep obstacles visible
                row.append('?' if v < 0 else '#' if v >= 99 else '+' if v >= 50
                           else '.' if v >= 1 else ' ')
            print(''.join(row).rstrip())
        self.done = True


rclpy.init()
node = Dump()
for _ in range(20):
    if node.done:
        break
    rclpy.spin_once(node, timeout_sec=1.0)
if not node.done:
    print('no costmap received in 20 s (is Nav2 active and the pose set?)')
node.destroy_node()
rclpy.try_shutdown()
