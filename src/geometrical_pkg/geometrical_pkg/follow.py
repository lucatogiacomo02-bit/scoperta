#!/usr/bin/env python3
import rclpy                                                                # type: ignore
from rclpy.node import Node                                                 # type: ignore
import threading
import math
import time

from geometry_msgs.msg import Twist                                         # type: ignore
from nav_msgs.msg import Odometry                                           # type: ignore
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy    # type: ignore


# -------------------------------------------------------------------
# Utility functions
# -------------------------------------------------------------------
def quaternion_to_yaw(qx, qy, qz, qw):
    """Compute yaw (rotation around Z) from quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Normalize angle to [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


# -------------------------------------------------------------------
# Main node
# -------------------------------------------------------------------
class Turtlebot3SquarePath(Node):
    def __init__(self):
        super().__init__('turtlebot3_square_path_node')

        # QoS for odometry + velocity
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers/Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos)

        # Parameters
        self.rate_hz           = 20.0

        # Odometry state
        self.x = None
        self.y = None
        self.yaw = None

        # Control flag
        self.is_running = True

        self.get_logger().info('Node initialized.')

    # -------------------------------------------------------------------
    # Callbacks and basic helpers
    # -------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        """Extract odometry pose."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def publish_twist(self, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.cmd_pub.publish(t)

    def publish_stop(self):
        self.publish_twist(0.0, 0.0)

    def wait_for_odom(self, timeout=5.0):
        """Wait until x, y, yaw are received."""
        start = time.time()
        while rclpy.ok() and (self.x is None or self.yaw is None):
            if time.time() - start > timeout:
                return False
            time.sleep(0.05)
        return True

    # -------------------------------------------------------------------
    # Motion primitives
    # -------------------------------------------------------------------
    def move_distance(self, distance, linear_vel):
        """Move forward by given distance using odometry."""
        if not self.wait_for_odom():
            self.get_logger().warn('No odom — aborting move_distance().')
            return

        start_x, start_y = self.x, self.y
        target = abs(distance)
        direction = 1.0 if distance >= 0 else -1.0
        rate_dt = 1.0 / self.rate_hz

        self.get_logger().info(f'Moving {("forward" if direction > 0 else "backward")} {target:.2f} m')

        while rclpy.ok() and self.is_running:
            dx = self.x - start_x
            dy = self.y - start_y
            traveled = math.hypot(dx, dy)

            if traveled >= target:
                break

            remaining = max(target - traveled, 0.0)
            speed = min(linear_vel, remaining * 1.0 + 0.05)

            self.publish_twist(linear=direction * speed)
            time.sleep(rate_dt)

        self.publish_stop()
        time.sleep(0.05)

    def rotate_angle(self, angle_rad, angular_vel):
        """Rotate by angle_rad (rad) using odometry yaw feedback."""
        if not self.wait_for_odom():
            self.get_logger().warn('No odom — aborting rotate_angle().')
            return

        start_yaw = self.yaw
        target_yaw = normalize_angle(start_yaw + angle_rad)
        direction = 1.0 if angle_rad >= 0 else -1.0
        rate_dt = 1.0 / self.rate_hz

        self.get_logger().info(f'Rotating {math.degrees(angle_rad):.1f} degrees')

        while rclpy.ok() and self.is_running:
            if self.yaw is None:
                time.sleep(rate_dt)
                continue

            error = normalize_angle(target_yaw - self.yaw)
            if abs(error) < math.radians(1.5):
                break

            k = 1.2
            angular_speed = min(angular_vel, max(0.05, abs(k * error)))

            self.publish_twist(angular=direction * angular_speed)
            time.sleep(rate_dt)

        self.publish_stop()
        time.sleep(0.05)

    # -------------------------------------------------------------------
    # High-level trajectories
    # -------------------------------------------------------------------
    def run_square_path(self, side_length, duration, per_rot_duration=0.3):
        """Execute a 4-sided square path."""
        if not self.wait_for_odom(timeout=10.0):
            self.get_logger().error('No odom received — cannot start square.')
            return

        self.get_logger().info('Starting square path.')

        n_sides = 4
        angle = math.pi / 2

        # Time allocation
        total_rot_time = per_rot_duration * duration
        total_straight_time = duration - per_rot_duration

        rot_time = total_rot_time / n_sides
        straight_time = total_straight_time / n_sides

        # Velocities
        linear_vel = side_length / straight_time
        angular_vel = angle / rot_time

        for i in range(n_sides):
            if not rclpy.ok() or not self.is_running:
                break

            self.get_logger().info(f'Side {i+1}/4: move {side_length:.2f} m')
            self.move_distance(side_length, linear_vel)

            self.get_logger().info(f'Side {i+1}/4: rotate 90°')
            self.rotate_angle(angle, angular_vel)

        self.publish_stop()
        self.get_logger().info('Completed square path.')

    def follow_polygon(self, n_edges, side_length, duration, per_rot_duration=0.3):
        if not self.wait_for_odom(timeout=10.0):
            self.get_logger().error('No odom — cannot start polygon.')
            return

        self.get_logger().info(f'Starting polygon with {n_edges} edges.')

        total_rot_time = per_rot_duration * duration
        total_straight_time = duration - per_rot_duration

        rot_time = total_rot_time / n_edges
        straight_time = total_straight_time / n_edges

        external_angle = 2.0 * math.pi / n_edges

        linear_vel = side_length / straight_time
        angular_vel = external_angle / rot_time

        for i in range(n_edges):
            if not rclpy.ok() or not self.is_running:
                break

            self.move_distance(side_length, linear_vel)
            self.rotate_angle(external_angle, angular_vel)

        self.publish_stop()

    def follow_circle(self, radius, duration):
        if not self.wait_for_odom(timeout=10.0):
            self.get_logger().error('No odom — cannot start circle.')
            return

        self.get_logger().info(f'Starting circular path: radius={radius} m, duration={duration}s')

        linear_vel = 2.0 * math.pi * radius / duration
        angular_vel = 2.0 * math.pi / duration

        rate_dt = 1.0 / self.rate_hz
        steps = int(duration * self.rate_hz)

        for _ in range(steps):
            if not rclpy.ok() or not self.is_running:
                break

            self.publish_twist(linear=linear_vel, angular=angular_vel)
            time.sleep(rate_dt)

        self.publish_stop()
        self.get_logger().info('Completed circular path.')


# -------------------------------------------------------------------
# Main entry
# -------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3SquarePath()

    # spin in background so odom updates
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        # node.run_square_path(side_length=0.5, duration=12.0)
        node.follow_polygon(n_edges=6, side_length=0.4, duration=10.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.is_running = False
        node.publish_stop()

        node.get_logger().info('Shutting down.')

        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()

