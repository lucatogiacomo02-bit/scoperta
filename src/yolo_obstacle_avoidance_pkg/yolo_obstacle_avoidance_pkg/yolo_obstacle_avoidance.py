#!/usr/bin/env python3
import os
os.environ["TORCH_CPP_LOG_LEVEL"] = "ERROR"

import time
import cv2
import numpy as np
import rclpy                                    # type: ignore
from rclpy.node import Node                     # type: ignore
            
from sensor_msgs.msg import Image, LaserScan    # type: ignore
from cv_bridge import CvBridge                  # type: ignore
from geometry_msgs.msg import Twist             # type: ignore

from vision_msgs.msg import (                   # type: ignore
    Detection2D,
    ObjectHypothesisWithPose,
    BoundingBox2D
)

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class YoloObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("limo_yolo")

        # ----------------------------------------------------
        # Parameters
        # ----------------------------------------------------
        self.declare_parameter("image_topic", "/image")
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("score_thresh", 0.5)
        self.declare_parameter("filter_class", "elephant")
        self.declare_parameter("stop_area_ratio", 0.10)
        self.declare_parameter("offset_distance", 0.03)

        self.img_topic = self.get_parameter("image_topic").value
        self.model_path = self.get_parameter("model").value
        self.score_thresh = self.get_parameter("score_thresh").value
        self.filter_class = self.get_parameter("filter_class").value.lower()
        self.stop_ratio = self.get_parameter("stop_area_ratio").value
        self.offset_distance = self.get_parameter("offset_distance").value

        # Internal state
        self.bridge = CvBridge()
        self.model = None
        self.model_names = {}

        self.last_detections = []

        self.target_box_area_ratio = 0.0
        self.target_detected = False
        self.target_confidence = 0.0
        self.target_last_seen_time = time.time()
        self.stopped = False   

        self.current_distance = np.inf
        self.obstacle_detected = False
        self.ranges = []

        self.latest_depth = None

        # ----------------------------------------------------
        # ROS I/O
        # ----------------------------------------------------
        self.sub_image = self.create_subscription(
            Image, self.img_topic, self.on_image, 10
        )

        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.on_scan, 10
        )

        self.cmd_pub = self.create_publisher(
            Twist, "/cmd_vel", 10
        )

        self.sub_depth = self.create_subscription(
            Image,
            "/camera/depth/image_raw",
            self.on_depth,
            10
        )

        # Control loop (20 Hz)
        self.control_timer = self.create_timer(
            0.05, self.control_loop
        )

        # Load YOLO
        self.load_model()
        self.get_logger().info(
            f"Node loaded successfully. Tracking: {self.filter_class}"
        )

    # ----------------------------------------------------
    # LOAD YOLO MODEL
    # ----------------------------------------------------
    def load_model(self):
        if YOLO is None:
            raise RuntimeError("Ultralytics YOLO not installed")

        t0 = time.time()
        self.model = YOLO(self.model_path)
        self.model.fuse()
        self.model_names = self.model.names

        dt = time.time() - t0
        self.get_logger().info(
            f"Loaded YOLO model '{self.model_path}' in {dt:.2f}s"
        )

    # ----------------------------------------------------
    # IMAGE CALLBACK
    # ----------------------------------------------------
    def on_image(self, msg: Image):

        # Convert ROS → OpenCV
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        h, w = img_bgr.shape[:2]
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

        # Run YOLO inference
        results = self.model.predict(
            img_rgb,
            imgsz=320,
            conf=self.score_thresh,
            verbose=False
        )

        if not results:
            return

        r = results[0]
        if r.boxes is None:
            return

        boxes = r.boxes
        xyxy = boxes.xyxy.cpu().numpy()
        conf = boxes.conf.cpu().numpy()
        cls_ids = boxes.cls.cpu().numpy()

        
        self.detect_target(
            xyxy, conf, cls_ids,
            h, w,
            target_class=self.filter_class,
            threshold_score=self.score_thresh
        )

    def on_depth(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warning(f"Depth cv_bridge failed: {e}")


    # LIDAR callback
    def on_scan(self, msg: LaserScan):
        # Get distances
        ranges = np.array(msg.ranges)

        # Update internal field
        self.ranges = ranges

        # Reduce range to -30/+30 degrees only
        n = len(ranges)
        center = n // 2
        window = ranges[center - 30 : center + 30]

        # Get the minimum distance in that range
        self.current_distance = np.nanmin(window)   # distance to closest object
        


    def detect_target(self, xyxy, conf, cls_ids, image_h, image_w,
                    target_class, threshold_score):

        for i in range(len(xyxy)):
            x1, y1, x2, y2 = xyxy[i]
            score = float(conf[i])
            cls_name = str(self.model_names.get(int(cls_ids[i]), "unknown")).lower()

            # Print detections
            self.get_logger().info(f"Detected: {cls_name} (score={score:.2f})")

            # Skip detection if the class is wrong
            if cls_name != target_class:
                continue
            
            # Skip if score is too low
            if not self.target_detected and score < threshold_score:
                continue
            
            # Relax threshold if target is already detected 
            if self.target_detected and score < (threshold_score / 2):
                continue

            # Valid target is found
            self.target_confidence = score
            self.target_last_seen_time = time.time()

            # Compute area ratio
            box_area = (x2 - x1) * (y2 - y1)
            self.target_box_area_ratio = box_area / (image_h * image_w)

            self.target_detected = True

            return  # Only use the first valid detection
        
    def detect_obstacle(self, threshold_dist=0.4):
        
        # Check whether an obstacle is in front
        if not self.target_detected:
            if self.current_distance < threshold_dist:
                self.obstacle_detected = True
                return
            
        if self.target_detected:
            if np.abs(self.latest_depth - self.current_distance) > self.offset_distance:
                self.obstacle_detected = True
                return
        
        self.obstacle_detected = False

    # Control loop
    def control_loop(self):
        now = time.time()

        if self.stopped:
            return
        
        # Detect obstacles
        self.detect_obstacle()

        # If obstacle detected, turn
        if self.obstacle_detected:

            # Identify most free direction
            n = len(self.ranges)
            center = n // 2

            left_window = self.ranges[ : (center - 30) ]
            right_window = self.ranges[ (center + 30) : ]

            left_min_distance = np.nanmin(left_window)      # min distance on the left half
            right_min_distance = np.nanmin(right_window)    # min distance on the right half

            if left_min_distance > right_min_distance:
                direction = 1   
            else:
                direction = -1

            # Rotate to avoid obstacle
            self.publish_twist(0.0, 0.5 * direction)

            return


        # If target not seen for 1s -> consider lost
        if now - self.target_last_seen_time > 1:
            self.target_detected = False
            self.latest_depth = None
            

        # If target not detected, rotate in place
        if not self.target_detected:
            self.stopped = False
            self.publish_twist(0.0, 0.4)
            return
        
        # If close enough to the target, stop
        if self.latest_depth is not None and self.target_detected:
            dist = self.latest_depth[self.target_cy, self.target_cx]

            if dist < self.safe_distance:
                self.publish_stop()
                self.stopped = True

                return       

        # Otherwise approach
        self.stopped = False
        self.publish_twist(0.4, 0.0)

    # ----------------------------------------------------
    # MOVEMENT HELPERS
    # ----------------------------------------------------
    def publish_twist(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def publish_stop(self):
        self.publish_twist(0.0, 0.0)


# ----------------------------------------------------
# MAIN
# ----------------------------------------------------
def main():
    rclpy.init()
    node = LimoYoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()   # ensure motors stop
        rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
