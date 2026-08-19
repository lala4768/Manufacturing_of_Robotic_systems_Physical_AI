import time

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
        self.current_twist.linear.x = 0.4
        self.current_twist.angular.z = 0.0

        self.timer = self.create_timer(0.1, self.timer_callback)

        # -------------------------
        # maneuver lock (도리도리 방지: 한 번 돌기로 정하면 commit)
        # -------------------------
        self.avoidance_end_time = 0.0
        self.avoid_duration = 0.5          # ≈ 34° commit @ 1.5 rad/s

        # 방향 기억: +1 = 좌(angular +), -1 = 우(angular -)
        # 이 코스는 왼쪽으로 도니까 기본값 좌
        self.last_turn = 1

        self.get_logger().info(
            "[FIXED] Front-lidar (forward-only ROI) obstacle avoidance started"
        )

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
        # 좌표축 규약 (left/right 뒤집은 버전: +row=우, -row=좌)
        # =================================================
        w_side = m2p(0.35)

        zA_near, zA_far = m2p(0.05), m2p(0.25)   # near (emergency)
        zB_near, zB_far = m2p(0.25), m2p(2.0)    # mid  (avoid)

        def cnt(r1, r2, c1, c2):
            roi = self._safe_slice(grid, r1, r2, c1, c2)
            return int(np.sum(roi == 100)) if roi.size else 0

        # FRONT-A (near) -- 전방만
        cntA_center = cnt(cy - w_side, cy + w_side, cx + zA_near, cx + zA_far)
        cntA_left   = cnt(cy - 2 * w_side, cy - w_side, cx + zA_near, cx + zA_far)
        cntA_right  = cnt(cy + w_side, cy + 2 * w_side, cx + zA_near, cx + zA_far)

        # FRONT-B (mid) -- 전방만
        cntB_center = cnt(cy - w_side, cy + w_side, cx + zB_near, cx + zB_far)
        cntB_left   = cnt(cy - 2 * w_side, cy - w_side, cx + zB_near, cx + zB_far)
        cntB_right  = cnt(cy + w_side, cy + 2 * w_side, cx + zB_near, cx + zB_far)

        # 셀 1~2개 노이즈 무시
        threshold = max(3, int(0.008 / (res * res)))

        now = time.time()

        # =================================================
        # P1: MANEUVER LOCK (도리도리 방지 - commit한 회전 유지)
        # =================================================
        if now < self.avoidance_end_time:
            # 직전에 세팅된 twist 그대로 publish
            return

        # =================================================
        # P2: EMERGENCY (VERY CLOSE FRONT)
        # =================================================
        if cntA_center > threshold:

            self.current_twist.linear.x = 0.0

            diff = cntA_left - cntA_right
            if diff > 2:                 # 왼쪽이 더 막힘 -> 우회전
                self.last_turn = -1
            elif diff < -2:              # 오른쪽이 더 막힘 -> 좌회전
                self.last_turn = 1
            # 비슷하면 직전 방향 유지

            self.current_twist.angular.z = 1.0 * self.last_turn
            return

        # =================================================
        # P3: AVOIDANCE (MID RANGE)
        # =================================================
        if cntB_center > threshold:

            # 이번 회전을 commit (lock)
            self.avoidance_end_time = now + self.avoid_duration

            self.current_twist.linear.x = 0.1   # creep forward (호 그리며 탈출)

            diff = cntB_left - cntB_right
            if diff > 2:
                self.last_turn = -1
            elif diff < -2:
                self.last_turn = 1

            self.current_twist.angular.z = 1.5 * self.last_turn
            return

        # =================================================
        # P4: STRAIGHT
        # =================================================
        self.current_twist.linear.x = 0.45
        self.current_twist.angular.z = 0.0

    # =====================================================
    def timer_callback(self):
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
