#!/usr/bin/env python3
"""
Target Detector Node
目标检测节点

Runs YOLO target detection and ArUco marker detection (marker mounted on the
ground robot) on the UAV's D436 color+depth stream, and publishes the
detected target's position relative to the marker. Does NOT convert this
into the map frame — that is the job of a separate coordinate-bridge node.
在无人机 D436 相机的彩色+深度图像上同时跑 YOLO 目标检测和 ArUco 标记检测
(标记贴在小车上),发布目标相对标记的位置。不做地图系转换——那是另一个
坐标桥接节点的工作。

Usage / 用法:
    ros2 run uav_vision target_detector_node
"""

import os
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from ultralytics import YOLO

# ultralytics resolves a bare weight filename (e.g. "yolov8m.pt") relative to
# the current working directory, so it can (and did) download a fresh copy
# into whatever directory happened to be cwd when the node was launched.
# Force a single fixed location inside the repo instead (kept out of git via
# the *.pt rule in .gitignore).
DEFAULT_WEIGHTS_DIR = os.path.expanduser(
    '~/01_SAP/Air-Ground-sar/ros2_ws/src/uav_vision/weights')


class TargetDetectorNode(Node):

    def __init__(self):
        super().__init__('target_detector_node')

        self.declare_parameter('color_image_topic', '/camera/camera/color/image_raw')
        # realsense2_camera only publishes pixel-aligned depth on this topic
        # when launched with align_depth.enable:=true; the plain
        # depth/image_rect_raw topic is NOT aligned to the color frame.
        self.declare_parameter(
            'depth_image_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('target_class_id', 0)
        self.declare_parameter('detection_confidence_threshold', 0.4)
        # shape filter: known distractor objects in the scene (e.g. a small
        # drone model) get boxed taller-than-wide, while the real target is
        # consistently wider-than-tall. Reject candidates outside this range.
        self.declare_parameter('min_aspect_ratio', 1.0)
        self.declare_parameter('max_aspect_ratio', 5.0)
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 2.5)
        self.declare_parameter('marker_max_age_sec', 2.0)
        self.declare_parameter('marker_frame_id', 'aruco_marker')
        self.declare_parameter('model_path', 'yolov8m.pt')
        # tiled inference: the target is often a small fraction of the full
        # frame, which gets lost once YOLO downsamples the whole image to its
        # input size. Slicing the frame into overlapping tiles close to
        # YOLO's native resolution recovers effective detail on small targets.
        self.declare_parameter('tile_size', 256)
        self.declare_parameter('tile_overlap', 0.4)
        # for a fixed (non-flying) camera, one confident detection is enough —
        # stop processing further frames once the first one succeeds instead
        # of re-running the expensive tiled pass on every incoming frame.
        self.declare_parameter('single_shot', False)
        # if set, every tile is saved to this directory with its YOLO
        # detections drawn on it, for visual debugging. Empty = disabled.
        self.declare_parameter('save_tile_debug_dir', '')

        self.marker_id = self.get_parameter('marker_id').value
        self.marker_size = self.get_parameter('marker_size').value
        self.target_class_id = self.get_parameter('target_class_id').value
        self.detection_confidence_threshold = self.get_parameter(
            'detection_confidence_threshold').value
        self.min_aspect_ratio = self.get_parameter('min_aspect_ratio').value
        self.max_aspect_ratio = self.get_parameter('max_aspect_ratio').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.marker_max_age_sec = self.get_parameter('marker_max_age_sec').value
        self.marker_frame_id = self.get_parameter('marker_frame_id').value
        self.tile_size = self.get_parameter('tile_size').value
        self.tile_overlap = self.get_parameter('tile_overlap').value
        self.single_shot = self.get_parameter('single_shot').value
        self.save_tile_debug_dir = self.get_parameter('save_tile_debug_dir').value
        if self.save_tile_debug_dir:
            os.makedirs(self.save_tile_debug_dir, exist_ok=True)
        self._done = False
        self._frame_index = 0
        model_path = self.get_parameter('model_path').value

        half = self.marker_size / 2.0
        self._marker_object_points = np.array([
            [-half,  half, 0.0],   # top-left
            [ half,  half, 0.0],   # top-right
            [ half, -half, 0.0],   # bottom-right
            [-half, -half, 0.0],   # bottom-left
        ], dtype=np.float32)

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.camera_matrix = None
        self.dist_coeffs = None
        self.depth_image = None
        self.last_marker_rvec = None
        self.last_marker_tvec = None
        self.last_marker_position_cam = None
        self.last_marker_time = None
        self.last_marker_corners = None
        self.last_marker_ids = None

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        if not os.path.isabs(model_path) and os.path.dirname(model_path) == '':
            os.makedirs(DEFAULT_WEIGHTS_DIR, exist_ok=True)
            model_path = os.path.join(DEFAULT_WEIGHTS_DIR, model_path)

        self.get_logger().info(f'Loading YOLO model ({model_path})...')
        self.yolo = YOLO(model_path)
        self.get_logger().info('YOLO model loaded.')

        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_camera_info, 10)
        self.create_subscription(
            Image, self.get_parameter('depth_image_topic').value,
            self._on_depth, 10)
        self.create_subscription(
            Image, self.get_parameter('color_image_topic').value,
            self._on_color, 10)

        self.pub = self.create_publisher(PoseStamped, '/camera/target_pose_raw', 10)
        # Detection-overlay image (bounding box + marker axes) for a live
        # dashboard view - published on every successful detection,
        # independent of save_tile_debug_dir (which is only for saving
        # frames to disk for offline debugging). JPEG-compressed rather than
        # raw: rosbridge falls behind and silently drops most frames of a
        # raw 1280x720 image at even ~1-2Hz (each frame is several MB once
        # base64-encoded into JSON) - a compressed frame is a few dozen KB,
        # which rosbridge keeps up with easily.
        # 带检测框/标记坐标轴的图像,给仪表盘实时显示用——每次检测成功都会
        # 发布,跟save_tile_debug_dir(只是为了离线调试存到磁盘)是两回事。
        # 用JPEG压缩而不是原始像素:1280x720的原始图像哪怕只有1-2Hz,
        # base64编码进JSON后每帧都有好几MB,rosbridge跟不上、会悄悄丢掉
        # 大部分帧——压缩后每帧只有几十KB,rosbridge处理起来轻松很多。
        self.pub_annotated = self.create_publisher(CompressedImage, '/camera/annotated_image/compressed', 10)

        self.get_logger().info(
            'Target detector ready. Waiting for camera_info and image frames...')

    def _on_camera_info(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def _on_depth(self, msg: Image):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        with self.lock:
            self.depth_image = depth

    def _on_color(self, msg: Image):
        if self._done:
            return
        if self.camera_matrix is None:
            return

        with self.lock:
            depth = None if self.depth_image is None else self.depth_image.copy()
        if depth is None:
            return

        color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        now = self.get_clock().now()

        self._update_marker_observation(color, depth, now)

        if self.last_marker_time is None:
            return
        marker_age = (now - self.last_marker_time).nanoseconds * 1e-9
        if marker_age > self.marker_max_age_sec:
            return

        detection = self._detect_target_box(color)
        if detection is None:
            return
        box, conf = detection

        target_pose = self._backproject_to_marker_frame(box, depth)
        if target_pose is None:
            return

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = self.marker_frame_id
        pose.pose.position.x = float(target_pose[0])
        pose.pose.position.y = float(target_pose[1])
        pose.pose.position.z = float(target_pose[2])
        pose.pose.orientation.w = 1.0
        self.pub.publish(pose)
        self.get_logger().info(
            f'Target published: x={target_pose[0]:.2f} y={target_pose[1]:.2f} '
            f'z={target_pose[2]:.2f} conf={conf:.2f} box={[round(v) for v in box.tolist()]} '
            f'(relative to {self.marker_frame_id})')

        annotated = self._build_annotated_image(color, box, conf, target_pose)
        annotated_msg = self.bridge.cv2_to_compressed_imgmsg(annotated, dst_format='jpg')
        annotated_msg.header.stamp = now.to_msg()
        self.pub_annotated.publish(annotated_msg)

        if self.save_tile_debug_dir:
            self._save_full_annotated(annotated)

        if self.single_shot:
            self._done = True
            self.get_logger().info('single_shot=true: detection complete, no longer processing frames.')

    def _update_marker_observation(self, color, depth, now):
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None or self.marker_id not in ids.flatten():
            return

        idx = list(ids.flatten()).index(self.marker_id)
        # solvePnP gives orientation (rvec) and a geometry-only translation
        # (tvec) that does NOT go through the depth sensor at all. Using
        # tvec for position while the target's position comes from the
        # depth sensor means the two don't share the same systematic depth
        # bias, so it can't cancel out when we take "target relative to
        # marker". Instead, keep only the rotation from solvePnP and get the
        # marker's own position the same way the target's is obtained — via
        # the depth sensor — so a common depth bias cancels in the subtraction.
        success, rvec, tvec = cv2.solvePnP(
            self._marker_object_points, corners[idx][0],
            self.camera_matrix, self.dist_coeffs)
        if not success:
            return

        marker_center_px = corners[idx][0].mean(axis=0)
        marker_position_cam = self._pixel_depth_to_camera_point(
            marker_center_px[0], marker_center_px[1], depth)
        if marker_position_cam is None:
            return

        self.last_marker_rvec = rvec
        self.last_marker_tvec = tvec
        self.last_marker_position_cam = marker_position_cam
        self.last_marker_time = now
        self.last_marker_corners = corners
        self.last_marker_ids = ids

    def _tile_positions(self, size):
        tile = self.tile_size
        if size <= tile:
            return [0]
        stride = max(int(tile * (1 - self.tile_overlap)), 1)
        positions = list(range(0, size - tile + 1, stride))
        last_edge = size - tile
        if positions[-1] != last_edge:
            # snap instead of appending a near-duplicate tile when the
            # strided grid already lands close enough to the edge
            if last_edge - positions[-1] < stride * 0.5:
                positions[-1] = last_edge
            else:
                positions.append(last_edge)
        return positions

    def _iter_tile_bounds(self, width, height):
        for y0 in self._tile_positions(height):
            for x0 in self._tile_positions(width):
                yield (x0, y0, min(x0 + self.tile_size, width),
                       min(y0 + self.tile_size, height))

    def _detect_target_box(self, color):
        h, w = color.shape[:2]
        best_conf = 0.0
        best_box = None
        best_tile_debug = None
        self._frame_index += 1
        for tile_index, (x0, y0, x1, y1) in enumerate(self._iter_tile_bounds(w, h)):
            tile = color[y0:y1, x0:x1]
            results = self.yolo(tile, verbose=False)
            for result in results:
                for box in result.boxes:
                    if int(box.cls[0]) != self.target_class_id:
                        continue
                    xyxy = box.xyxy[0].cpu().numpy()
                    box_w = xyxy[2] - xyxy[0]
                    box_h = xyxy[3] - xyxy[1]
                    aspect_ratio = box_w / box_h if box_h > 0 else 0.0
                    if not (self.min_aspect_ratio <= aspect_ratio <= self.max_aspect_ratio):
                        continue
                    conf = float(box.conf[0])
                    if conf > best_conf:
                        best_conf = conf
                        best_box = np.array([
                            xyxy[0] + x0, xyxy[1] + y0,
                            xyxy[2] + x0, xyxy[3] + y0,
                        ])
                        best_tile_debug = (tile, results, tile_index, x0, y0)

        if best_box is None or best_conf < self.detection_confidence_threshold:
            return None
        if self.save_tile_debug_dir and best_tile_debug is not None:
            self._save_tile_debug(*best_tile_debug)
        return best_box, best_conf

    def _save_tile_debug(self, tile, results, tile_index, x0, y0):
        annotated = tile.copy()
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(
                    annotated, f'cls={cls} {conf:.2f}', (x1, max(y1 - 5, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        filename = f'frame{self._frame_index:04d}_tile{tile_index:02d}_x{x0}_y{y0}.png'
        cv2.imwrite(os.path.join(self.save_tile_debug_dir, filename), annotated)

    def _build_annotated_image(self, color, box, conf, target_pose):
        annotated = color.copy()
        if self.last_marker_corners is not None:
            cv2.aruco.drawDetectedMarkers(
                annotated, self.last_marker_corners, self.last_marker_ids)
        cv2.drawFrameAxes(
            annotated, self.camera_matrix, self.dist_coeffs,
            self.last_marker_rvec, self.last_marker_tvec, self.marker_size * 0.7)

        x1, y1, x2, y2 = [int(v) for v in box.tolist()]
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 3)
        cv2.putText(
            annotated, f'person {conf:.2f}', (x1, max(y1 - 10, 15)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        h, w = annotated.shape[:2]
        label = (f'target rel. {self.marker_frame_id}: '
                  f'x={target_pose[0]:.2f} y={target_pose[1]:.2f} z={target_pose[2]:.2f} m')
        cv2.rectangle(annotated, (0, h - 40), (w, h), (255, 255, 255), -1)
        cv2.putText(annotated, label, (10, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return annotated

    def _save_full_annotated(self, annotated):
        filename = f'frame{self._frame_index:04d}_full.png'
        cv2.imwrite(os.path.join(self.save_tile_debug_dir, filename), annotated)

    def _pixel_depth_to_camera_point(self, u, v, depth):
        h, w = depth.shape
        u_px = int(np.clip(round(u), 0, w - 1))
        v_px = int(np.clip(round(v), 0, h - 1))

        d = float(depth[v_px, u_px]) * 0.001  # mm -> m
        if d < self.min_depth or d > self.max_depth:
            self.get_logger().warn(
                f'Depth {d:.2f}m at ({u_px},{v_px}) out of range '
                f'[{self.min_depth}, {self.max_depth}]. Skipping frame.',
                throttle_duration_sec=1.0)
            return None

        # The pinhole formula below assumes an ideal (distortion-free) lens.
        # Undistort the pixel first so (u, v) reflects where that lens
        # distortion coefficients say the point would land on an ideal
        # sensor — otherwise points away from the image center get a
        # systematic position error.
        undistorted = cv2.undistortPoints(
            np.array([[[float(u), float(v)]]]),
            self.camera_matrix, self.dist_coeffs, P=self.camera_matrix)
        u_corr, v_corr = undistorted[0, 0]

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        x_cam = (u_corr - cx) * d / fx
        y_cam = (v_corr - cy) * d / fy
        return np.array([x_cam, y_cam, d])

    def _backproject_to_marker_frame(self, box, depth):
        u = (box[0] + box[2]) / 2
        v = (box[1] + box[3]) / 2
        point_cam = self._pixel_depth_to_camera_point(u, v, depth)
        if point_cam is None:
            return None

        # Both the marker's own position and the target's position are now
        # derived from the depth sensor via the same pixel->camera-point
        # path, so a common depth-sensor bias cancels out here instead of
        # leaking into the relative position (see _update_marker_observation).
        R, _ = cv2.Rodrigues(self.last_marker_rvec)
        R_inv = R.T
        point_marker = R_inv @ (point_cam - self.last_marker_position_cam)
        return point_marker


def main(args=None):
    rclpy.init(args=args)
    node = TargetDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
