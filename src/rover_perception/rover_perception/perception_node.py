import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

FONT = cv2.FONT_HERSHEY_SIMPLEX


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.declare_parameter('detector', 'color')   # 'color' or 'yolo'
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('classes', '')         # e.g. "person,fire hydrant"; empty = any
        self.declare_parameter('min_area', 300)
        self.declare_parameter('depth_min', 0.1)
        self.declare_parameter('depth_max', 8.0)

        self.detector = self.get_parameter('detector').value
        self.conf = float(self.get_parameter('conf').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.classes = [c.strip() for c in
                        str(self.get_parameter('classes').value).split(',') if c.strip()]
        self.bridge = CvBridge()
        self.K = None
        self.others = []          # every YOLO detection this frame, for the debug image

        if self.detector == 'yolo':
            from ultralytics import YOLO   # imported lazily so colour mode stays light
            self.model = YOLO(self.get_parameter('model').value)
            self.get_logger().info(
                f'YOLO loaded ({self.get_parameter("model").value}), '
                f'conf={self.conf}, classes={self.classes or "any"}')

        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        rgb_sub = Subscriber(self, Image, '/camera/image_raw',
                             qos_profile=qos_profile_sensor_data)
        depth_sub = Subscriber(self, Image, '/camera/depth/image_raw',
                               qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], 5, 0.1)
        self.sync.registerCallback(self.on_images)

        self.point_pub = self.create_publisher(PointStamped, '/perception/target_camera', 10)
        self.label_pub = self.create_publisher(String, '/perception/target_label', 10)
        self.debug_pub = self.create_publisher(Image, '/perception/debug_image', 1)
        self.get_logger().info(f'perception_node up (detector={self.detector})')

    def on_info(self, msg):
        if self.K is None:
            self.K = list(msg.k)
            self.get_logger().info(
                f'intrinsics: fx={self.K[0]:.1f} fy={self.K[4]:.1f} '
                f'cx={self.K[2]:.1f} cy={self.K[5]:.1f}')

    # each detector returns (x, y, w, h, label, confidence, mask_or_None) or None
    def detect_color(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = (cv2.inRange(hsv, (0, 120, 40), (10, 255, 255)) |
                cv2.inRange(hsv, (170, 120, 40), (180, 255, 255)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < self.get_parameter('min_area').value:
            return None
        x, y, w, h = cv2.boundingRect(c)
        return x, y, w, h, 'red_object', 1.0, mask

    def detect_yolo(self, bgr):
        res = self.model.predict(bgr, imgsz=self.imgsz, conf=self.conf,
                                 device='cpu', verbose=False)[0]
        H, W = bgr.shape[:2]
        best = None
        self.others = []
        for b in res.boxes:
            name = res.names[int(b.cls[0])]
            conf = float(b.conf[0])
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(W - 1, x2), min(H - 1, y2)
            self.others.append((x1, y1, x2, y2, name, conf))
            if self.classes and name not in self.classes:
                continue
            if best is None or conf > best[5]:
                best = (x1, y1, x2 - x1, y2 - y1, name, conf, None)
        return best

    def target_depth(self, depth, det):
        x, y, w, h, _, _, mask = det
        dmin = self.get_parameter('depth_min').value
        dmax = self.get_parameter('depth_max').value
        if mask is not None:      # colour: use exactly the detected pixels
            roi = depth[y:y + h, x:x + w]
            sel = mask[y:y + h, x:x + w] > 0
        else:                     # YOLO: central 40% of the box, to avoid background at the edges
            x0, x1 = x + int(0.3 * w), x + int(0.7 * w)
            y0, y1 = y + int(0.3 * h), y + int(0.7 * h)
            roi = depth[y0:y1 + 1, x0:x1 + 1]
            sel = np.ones(roi.shape, bool)
        ok = sel & np.isfinite(roi) & (roi > dmin) & (roi < dmax)
        if ok.sum() < 10:
            return None
        return float(np.median(roi[ok]))

    def on_images(self, rgb_msg, depth_msg):
        if self.K is None:
            return
        t0 = time.perf_counter()
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')   # float32 metres
        debug = bgr.copy()

        t1 = time.perf_counter()
        det = self.detect_yolo(bgr) if self.detector == 'yolo' else self.detect_color(bgr)
        detect_ms = (time.perf_counter() - t1) * 1000.0

        for (x1, y1, x2, y2, name, conf) in self.others:   # everything YOLO saw, thin yellow
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 255), 1)
            cv2.putText(debug, f'{name} {conf:.2f}', (x1, max(12, y1 - 4)),
                        FONT, 0.4, (0, 255, 255), 1)

        if det is None:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))
            return
        x, y, w, h, label, conf, _ = det

        Z = self.target_depth(depth, det)
        if Z is None:
            self.get_logger().warn('detection has no valid depth', throttle_duration_sec=2.0)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))
            return

        # Pinhole back-projection: [X Y Z]^T = Z * K^-1 [u v 1]^T
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        u, v = x + w / 2.0, y + h / 2.0
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        pt = PointStamped()
        pt.header.stamp = depth_msg.header.stamp
        pt.header.frame_id = depth_msg.header.frame_id
        pt.point.x, pt.point.y, pt.point.z = X, Y, Z
        self.point_pub.publish(pt)
        self.label_pub.publish(String(data=label))

        cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(debug, (int(u), int(v)), 4, (0, 255, 0), -1)
        cv2.putText(debug, f'{label} {conf:.2f}  X={X:.2f} Y={Y:.2f} Z={Z:.2f} m',
                    (x, max(15, y - 8)), FONT, 0.5, (0, 255, 0), 1)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))

        total_ms = (time.perf_counter() - t0) * 1000.0
        self.get_logger().info(
            f'{label} {conf:.2f} px=({u:.0f},{v:.0f}) cam=({X:.3f},{Y:.3f},{Z:.3f}) m  '
            f'detect {detect_ms:.0f} ms  total {total_ms:.0f} ms',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
