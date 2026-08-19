import time
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist

import numpy as np


class SimObstacleDetector(Node):

    def __init__(self):
        super().__init__('real_obstacle_detector')

        # -------------------------
        # ROS
        # -------------------------
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/occupancy_map',
            self.map_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # -------------------------
        # command
        # -------------------------
        self.current_twist = Twist()
        self.current_twist.linear.x = 25.0
        self.current_twist.angular.z = 0.0

        # -------------------------
        # initial right-turn ignore
        # -------------------------
        self.start_time = time.time()
        self.ignore_right_turn_duration = 20.0

        self.timer = self.create_timer(0.1, self.timer_callback)

        # -------------------------
        # recovery
        # -------------------------
        self.recovery_end_time = 0.0
        self.recovery_duration = 1.0

        # -------------------------
        # turn accumulation
        # -------------------------
        self.turn_accum = 0.0
        self.recovery_triggered = False
        self.RECOVERY_THRESHOLD = math.radians(25)

        self.yaw_est = 0.0
        self.last_int_time = None
        self.last_yaw = 0.0

        self.get_logger().info(
            "[FIXED] Front-lidar symmetric obstacle avoidance started"
        )

    # =====================================================
    def _wrap(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    # =====================================================
    def _safe_slice(self, grid, r1, r2, c1, c2):
        h, w = grid.shape
        r1 = max(0, min(r1, h))
        r2 = max(0, min(r2, h))
        c1 = max(0, min(c1, w))
        c2 = max(0, min(c2, w))
        if r1 >= r2 or c1 >= c2:
            return np.array([])
        return grid[r1:r2, c1:c2]

    # =====================================================
    def map_callback(self, msg):

        res = msg.info.resolution
        if res <= 0:
            return

        grid = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

        cx = msg.info.width // 2
        cy = msg.info.height // 2

        def m2p(m):
            return max(1, int(m / res))

        # =================================================
        # FRONT WIDTH (핵심 수정)
        # =================================================
        w_front = m2p(0.25)     # 정면 폭
        w_side  = m2p(0.35)     # 좌우 폭

        # distances
        zA = m2p(0.25)          # near
        zB = m2p(3.0)          # mid

        def cnt(r1, r2, c1, c2):
            roi = self._safe_slice(grid, r1, r2, c1, c2)
            return int(np.sum(roi == 100)) if roi.size else 0

        # =================================================
        # 🔥 FRONT-A (CRITICAL ZONE)
        # =================================================
        cntA_center = cnt(
            cy - w_side,
            cy + w_side,
            cx - wA if (wA := zA) else cx - zA,
            cx + wA
        )

        # =================================================
        # FRONT-B (MID ZONE)
        # =================================================
        cntB_center = cnt(
            cy - w_side,
            cy + w_side,
            cx - zB,
            cx + zB
        )

        # =================================================
        # LEFT / RIGHT (direction decision)
        # =================================================
        cntA_left = cnt(
            cy + w_side,
            cy + 2 * w_side,
            cx - zA,
            cx + zA
        )

        cntA_right = cnt(
            cy - 2 * w_side,
            cy - w_side,
            cx - zA,
            cx + zA
        )

        cntB_left = cnt(
            cy + w_side,
            cy + 2 * w_side,
            cx - zB,
            cx + zB
        )

        cntB_right = cnt(
            cy - 2 * w_side,
            cy - w_side,
            cx - zB,
            cx + zB
        )

        now = time.time()

        # =================================================
        # RESET RECOVERY
        # =================================================
        if cntA_center > 0 or cntB_center > 0:
            self.turn_accum = 0.0
            self.recovery_triggered = False

        # =================================================
        # RECOVERY
        # =================================================
        if now < self.recovery_end_time:
            self.current_twist.linear.x = -0.4
            self.current_twist.angular.z = 0.8
            return

        # =================================================
        # EMERGENCY (VERY CLOSE FRONT)
        # =================================================
        if cntA_center > 0:

            self.current_twist.linear.x = 0.0

            diff = cntA_left - cntA_right

            if diff > 0:
                self.current_twist.angular.z = -1.5
            elif diff < 0:
                self.current_twist.angular.z = 1.5
            else:
                self.current_twist.angular.z = 0.5

            return

        # =================================================
        # AVOIDANCE (MID RANGE)
        # =================================================
        if cntB_center > 0:

            self.current_twist.linear.x = 0.4

            diff = cntB_left - cntB_right

            if diff > 0:
                self.current_twist.angular.z = -2.0
            elif diff < 0:
                self.current_twist.angular.z = 2.0
            else:
                self.current_twist.angular.z = 2.0

            return

        # =================================================
        # STRAIGHT
        # =================================================
        self.current_twist.linear.x = 15.0
        self.current_twist.angular.z = 0.0

        # =================================================
        # RECOVERY TRIGGER
        # =================================================
        if self.turn_accum > self.RECOVERY_THRESHOLD and not self.recovery_triggered:

            self.recovery_triggered = True
            self.recovery_end_time = now + self.recovery_duration

            self.get_logger().warning(
                "RECOVERY triggered by turn accumulation"
            )

    # =====================================================
    def timer_callback(self):

        now = time.time()

        if self.last_int_time is not None:

            dt = now - self.last_int_time

            self.yaw_est += self.current_twist.angular.z * dt
            self.yaw_est = self._wrap(self.yaw_est)

            delta = self._wrap(self.yaw_est - self.last_yaw)
            self.turn_accum += abs(delta)
            self.last_yaw = self.yaw_est

        self.last_int_time = now

        if time.time() - self.start_time < self.ignore_right_turn_duration:
            if self.current_twist.angular.z < 0:
                self.current_twist.angular.z = 0.0
                
        self.cmd_pub.publish(self.current_twist)


# =========================================================
def main(args=None):
    rclpy.init(args=args)
    node = SimObstacleDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Stopped")

    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
