'''
#!/usr/bin/env python3
import os
os.environ["TORCH_CPP_LOG_LEVEL"] = "ERROR"

import time
import cv2
import numpy as np
import rclpy                                        # type: ignore
from rclpy.node import Node                         # type: ignore

from sensor_msgs.msg import Image, LaserScan        # type: ignore
from cv_bridge import CvBridge                      # type: ignore
from geometry_msgs.msg import Twist                 # type: ignore

from vision_msgs.msg import (                       # type: ignore
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
    BoundingBox2D
)

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class LimoYoloNode(Node):
    def __init__(self):
        super().__init__("limo_yolo")

        # ----------------------------------------------------
        # Parameters
        # ----------------------------------------------------
        self.declare_parameter("image_topic", "/image")
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("score_thresh", 0.5)
        self.declare_parameter("filter_class", "elephant")
        self.declare_parameter("safe_distance", 0.4)

        self.img_topic = self.get_parameter("image_topic").value
        self.model_path = self.get_parameter("model").value
        self.score_thresh = self.get_parameter("score_thresh").value
        self.filter_class = self.get_parameter("filter_class").value.lower()
        self.safe_distance = self.get_parameter("safe_distance").value

        # ----------------------------------------------------
        # Internal state vars
        # ----------------------------------------------------
        self.bridge = CvBridge()
        self.model = None
        self.model_names = {}
        self.current_distance = None
        self.last_detections = []
        self.target_detected = False
        self.target_confidence = 0.0
        self.target_last_seen_time = time.time()

        # control loop (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # ----------------------------------------------------
        # ROS I/O
        # ----------------------------------------------------
        self.sub_image = self.create_subscription(
            Image, self.img_topic, self.on_image, 10
        )

        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.on_scan, 10
        )

        self.pub_det = self.create_publisher(
            Detection2DArray, "detections", 10
        )

        self.cmd_pub = self.create_publisher(
            Twist, "/cmd_vel", 10
        )

        # Main behavior timer (runs at 20 Hz)
        self.behavior_timer = self.create_timer(
            0.05, self.run_state_machine
        )

        self.load_model()

        self.get_logger().info(f"Limo YOLO node ready. Searching for {self.filter_class}")

    # ----------------------------------------------------
    # MODEL LOADING
    # ----------------------------------------------------
    def load_model(self):
        if YOLO is None:
            raise RuntimeError("Ultralytics not installed")

        t0 = time.time()
        self.model = YOLO(self.model_path)
        self.model.fuse()
        self.model_names = self.model.names
        self.get_logger().info(f"Loaded YOLO model in {time.time()-t0:.2f}s")

    # ----------------------------------------------------
    # CAMERA CALLBACK
    # ----------------------------------------------------
    def on_image(self, msg: Image):
        """Runs YOLO and stores latest detections."""
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

        results = self.model.predict(
            img_rgb,
            imgsz=320,
            conf=self.score_thresh,
            verbose=False
        )

        # Clear previous
        self.last_detections = []

        if not results:
            return

        r = results[0]
        if r.boxes is None:
            return

        boxes = r.boxes
        xyxy = boxes.xyxy.cpu().numpy()
        conf = boxes.conf.cpu().numpy()
        cls_ids = boxes.cls.cpu().numpy()

        for i in range(len(xyxy)):
            x1, y1, x2, y2 = xyxy[i]
            score = conf[i]
            cls_id = int(cls_ids[i])
            cls_name = str(self.model_names.get(cls_id, cls_id)).lower()

            if cls_name != self.filter_class:
                continue

            det = Detection2D()
            det.bbox = BoundingBox2D()

            det.bbox.center.position.x = float((x1 + x2) / 2)
            det.bbox.center.position.y = float((y1 + y2) / 2)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = cls_name
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)

            self.last_detections.append(det)

    # ----------------------------------------------------
    # LIDAR CALLBACK
    # ----------------------------------------------------
    def on_scan(self, msg: LaserScan):
        """Updates current forward distance."""
        # min distance in front 30 degrees window
        ranges = np.array(msg.ranges)
        valid = ranges[~np.isnan(ranges)]
        if len(valid) == 0:
            self.current_distance = None
        else:
            self.current_distance = float(np.min(valid))

    # ----------------------------------------------------
    # BEHAVIOR STATE MACHINE
    # ----------------------------------------------------
    def run_state_machine(self):

        if self.state == "SEARCH":
            self.search_behavior()

        elif self.state == "APPROACH":
            self.approach_behavior()

        elif self.state == "STOP":
            self.publish_stop()

    def control_loop(self):

        now = time.time()

        # If target lost, return to search mode
        if now - self.target_last_seen_time > 0.3:  # 300 ms without detection
            self.target_detected = False

        # -----------------------------------------------------
        # 2) If NO target → SEARCH MODE (spin)
        # -----------------------------------------------------
        if not self.target_detected:
            self.publish_twist(0.0, 0.6)  # spin left
            return

        # -----------------------------------------------------
        # 3) If target detected BUT no LIDAR yet
        # -----------------------------------------------------
        if self.current_distance is None:
            self.publish_stop()
            self.get_logger().info("Target found, waiting for LIDAR...")
            return

        # -----------------------------------------------------
        # 4) If close enough → STOP
        # -----------------------------------------------------
        if self.current_distance <= self.safe_distance:
            self.publish_stop()
            return

        # -----------------------------------------------------
        # 5) APPROACH target
        # -----------------------------------------------------
        self.publish_twist(0.4, 0.0)


    # ----------------------------------------------------
    # SEARCH MODE
    # ----------------------------------------------------
    def search_behavior(self):
        """Spin until target detected."""
        # spin in place
        self.publish_twist(0.0, 0.8)

        for det in self.last_detections:
            cls = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score

            self.get_logger().info(
                f"Detected {cls} score={score:.2f}"
            )

            if score >= self.score_thresh:
                self.get_logger().info("Target found → switching to APPROACH")
                self.state = "APPROACH"
                self.publish_stop()
                return

    # ----------------------------------------------------
    # APPROACH MODE
    # ----------------------------------------------------
    def approach_behavior(self):
        """Drive forward until safe distance reached."""
        if self.current_distance is None:
            self.get_logger().info("No LIDAR yet...")
            return

        if self.current_distance <= self.safe_distance:
            self.get_logger().info("Close enough — stopping")
            self.publish_stop()
            self.state = "STOP"
            return

        # move forward slowly
        self.publish_twist(0.5, 0.0)

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


def main():
    rclpy.init()
    node = LimoYoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
'''

#!/usr/bin/env python3
import os
os.environ["TORCH_CPP_LOG_LEVEL"] = "ERROR"

import time
import cv2
import numpy as np
import rclpy                               # type: ignore
from rclpy.node import Node                # type: ignore
            
from sensor_msgs.msg import Image          # type: ignore
from cv_bridge import CvBridge             # type: ignore
from geometry_msgs.msg import Twist        # type: ignore

from vision_msgs.msg import (              # type: ignore
    Detection2D,
    ObjectHypothesisWithPose,
    BoundingBox2D
)

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class LimoYoloNode(Node):
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

        self.img_topic = self.get_parameter("image_topic").value
        self.model_path = self.get_parameter("model").value
        self.score_thresh = self.get_parameter("score_thresh").value
        self.filter_class = self.get_parameter("filter_class").value.lower()
        self.stop_ratio = self.get_parameter("stop_area_ratio").value

        # ----------------------------------------------------
        # Internal state
        # ----------------------------------------------------
        self.bridge = CvBridge()
        self.model = None
        self.model_names = {}

        self.last_detections = []

        self.target_box_area_ratio = 0.0
        self.target_detected = False
        self.target_confidence = 0.0
        self.target_last_seen_time = time.time()
        self.stopped = False   

        self.latest_depth = None
        self.safe_distance = 1

        # ----------------------------------------------------
        # ROS I/O
        # ----------------------------------------------------
        self.sub_image = self.create_subscription(
            Image, self.img_topic, self.on_image, 10
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

        # To publish the image with bounding box
        self.pub_image = self.create_publisher(
            Image,
            "/yolo/annotated_image",
            10
        )



        # Control loop (20 Hz)
        self.control_timer = self.create_timer(
            0.05, self.control_loop
        )

        # Load YOLO
        self.load_model()
        self.get_logger().info(
            f"Limo YOLO node ready. Tracking: {self.filter_class}"
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

        # -------------------------------------------------------
        # Draw bounding box on image if target detected
        # -------------------------------------------------------
        if self.target_detected:

            x = int(self.target_cx)
            y = int(self.target_cy)

            
            for i in range(len(xyxy)):
                x1, y1, x2, y2 = xyxy[i]
                cls_name = str(self.model_names.get(int(cls_ids[i]), "unknown")).lower()

                if cls_name == self.filter_class:
                    x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

                    # Draw rectangle
                    cv2.rectangle(
                        img_bgr,
                        (x1, y1),
                        (x2, y2),
                        (0, 255, 0),
                        2
                    )

                    # Draw center point
                    cv2.circle(
                        img_bgr,
                        (self.target_cx, self.target_cy),
                        5,
                        (0, 0, 255),
                        -1
                    )

                    # Draw label
                    label = f"{self.filter_class} {self.target_confidence:.2f}"
                    cv2.putText(
                        img_bgr,
                        label,
                        (x1, max(y1 - 10, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

                    break

            # -------------------------------------------------------
            # Publish annotated image
            # -------------------------------------------------------
            out_msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
            self.pub_image.publish(out_msg)


    def on_depth(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warning(f"Depth cv_bridge failed: {e}")




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

            self.target_cx = int((x1 + x2) / 2)
            self.target_cy = int((y1 + y2) / 2)


            self.target_detected = True

            return  # Only use the first valid detection

    # Control loop
    def control_loop(self):
        now = time.time()

        if self.stopped:
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

        # If the bounding box is large enough → stop
        '''
        if self.target_box_area_ratio >= self.stop_ratio:
            if not self.stopped:
                self.get_logger().info(
                    f"Target close (bbox={self.target_box_area_ratio:.2f}) -> stopping..."
                )
                self.stopped = True
            self.publish_stop()
            return
        '''

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
