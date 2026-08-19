import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
import numpy as np


class SimObstacleDetector(Node):
    def __init__(self):
        super().__init__('sim_obstacle_detector')

        # ── Topics ──────────────────────────────────────────────────────────
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/occupancy_map', self.map_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── 10 Hz heartbeat so the controller never times-out ────────────
        # Pre-set cruise speed so the robot moves immediately on startup
        # instead of freezing until the first /occupancy_map message arrives.
        self.current_twist = Twist()
        self.current_twist.linear.x = 0.45   # start moving right away
        self.timer = self.create_timer(0.1, self.timer_callback)

        # ── Maneuver-lock: rotate 20° then re-check ──────────────────────
        # 20° = 0.349 rad; at turn speed 0.9 rad/s → lock lasts 0.39 s
        self.avoidance_end_time  = 0.0
        self.avoid_duration      = round((20 * 3.14159 / 180) / 0.9, 3)  # ≈ 0.39 s

        # ── Recovery (back-up when genuinely blocked) ────────────────────
        self.recovery_end_time   = 0.0
        self.recovery_duration   = 1.8   # s – reverse + pivot duration
        self.last_obstacle_time  = 0.0
        self.stuck_timeout       = 4.0   # s – declare stuck if avoiding > this

        # ── Turn memory ──────────────────────────────────────────────────
        self.last_turn = 0.8             # default: left (positive ω)
        self.current_state = "INIT"

        self.get_logger().info(
            "[SIM] Multi-zone obstacle avoidance node started.")

    # ────────────────────────────────────────────────────────────────────────
    def _safe_slice(self, grid, r1, r2, c1, c2):
        """Clamp slice indices to grid bounds; return sub-array (may be empty)."""
        H, W = grid.shape
        r1 = max(0, min(r1, H))
        r2 = max(0, min(r2, H))
        c1 = max(0, min(c1, W))
        c2 = max(0, min(c2, W))
        if r1 >= r2 or c1 >= c2:
            return np.array([])
        return grid[r1:r2, c1:c2]

    # ────────────────────────────────────────────────────────────────────────
    def map_callback(self, msg):
        res = msg.info.resolution
        if res <= 0:
            return

        grid = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))

        cx = msg.info.width  // 2   # robot column in grid
        cy = msg.info.height // 2   # robot row    in grid

        # ── Physical distance → pixel offsets ───────────────────────────
        # Zone A  (EMERGENCY): 0.05 m – 0.40 m ahead
        # Zone B  (AVOID)    : 0.40 m – 1.00 m ahead
        # Lateral half-widths: narrow (robot body ~0.30 m) & wide (~0.88 m)
        def m2p(meters):
            return max(1, int(meters / res))

        zA_near, zA_far = m2p(0.05), m2p(0.40)   # Emergency: 5–40 cm
        zB_near, zB_far = m2p(0.40), m2p(1.00)   # Avoid: 40 cm–1.0 m
        w_narrow        = m2p(0.30)               # Robot body half-width
        w_wide          = m2p(0.88)               # Safety bubble half-width

        # ── ROI helpers (forward = +col, left = +row, right = -row) ─────
        #
        #  In a local occupancy grid centred on the robot:
        #    col increases  →  robot's +X  (forward)
        #    row increases  →  robot's +Y  (left in ROS REP-103)
        #    row decreases  →  robot's -Y  (right)

        def cnt(r1, r2, c1, c2):
            roi = self._safe_slice(grid, r1, r2, c1, c2)
            return int(np.sum(roi == 100)) if roi.size else 0

        # Zone A (close)
        cntA_center = cnt(cy - w_narrow, cy + w_narrow, cx + zA_near, cx + zA_far)
        cntA_left   = cnt(cy + w_narrow, cy + w_wide,   cx + zA_near, cx + zA_far)
        cntA_right  = cnt(cy - w_wide,   cy - w_narrow, cx + zA_near, cx + zA_far)

        # Zone B (far)
        cntB_center = cnt(cy - w_narrow, cy + w_narrow, cx + zB_near, cx + zB_far)
        cntB_left   = cnt(cy + w_narrow, cy + w_wide,   cx + zB_near, cx + zB_far)
        cntB_right  = cnt(cy - w_wide,   cy - w_narrow, cx + zB_near, cx + zB_far)

        # Dynamic threshold proportional to cell area
        threshold = max(3, int(0.008 / (res * res)))

        current_time = time.time()

        # ════════════════════════════════════════════════════════════════
        # STATE MACHINE
        # ════════════════════════════════════════════════════════════════

        # ── Priority 0: RECOVERY (back-up + pivot) ───────────────────────
        if current_time < self.recovery_end_time:
            if self.current_state != "RECOVERY":
                self.get_logger().warning(
                    "RECOVERY: reversing to escape dead-end.")
                self.current_state = "RECOVERY"
            self.current_twist.linear.x  = -0.2
            self.current_twist.angular.z =  self.last_turn * 1.2
            return

        # ── Priority 1: MANEUVER LOCK (ignore flicker during turn) ───────
        if current_time < self.avoidance_end_time:
            if self.current_state != "FORCED_AVOID":
                self.get_logger().info(
                    f"Maneuver lock active ({self.avoid_duration:.1f}s)…")
                self.current_state = "FORCED_AVOID"
            # (keep whatever twist was set when the lock started)
            return

        # ── Priority 2: EMERGENCY (Zone A – very close) ──────────────────
        if cntA_center > threshold:
            if self.current_state != "EMERGENCY":
                self.get_logger().warning(
                    f"EMERGENCY! Obstacle <0.35 m  "
                    f"(L:{cntA_left} C:{cntA_center} R:{cntA_right})")
                self.current_state = "EMERGENCY"

            # If we have been stuck in avoidance for too long → recover
            if (self.last_obstacle_time > 0 and
                    current_time - self.last_obstacle_time > self.stuck_timeout):
                self.get_logger().warning("Stuck too long – triggering RECOVERY.")
                self.recovery_end_time = current_time + self.recovery_duration
                self.last_obstacle_time = 0.0
                return

            self.last_obstacle_time = self.last_obstacle_time or current_time

            # Stop linear motion; pivot away from denser side
            self.current_twist.linear.x = 0.0
            diff = cntA_left - cntA_right
            if diff > 2:                        # more obstacles on left  → turn right
                self.current_twist.angular.z = -1.0
                self.last_turn = -1.0
            elif diff < -2:                     # more obstacles on right → turn left
                self.current_twist.angular.z =  1.0
                self.last_turn =  1.0
            else:                               # symmetric → keep last direction
                self.current_twist.angular.z = self.last_turn
            return

        # ── Priority 3: AVOID (Zone B – medium range) ────────────────────
        if cntB_center > threshold:
            if self.current_state != "AVOIDING":
                self.avoidance_end_time  = current_time + self.avoid_duration
                self.last_obstacle_time  = current_time
                self.get_logger().warning(
                    f"Obstacle 0.40–1.00 m  "
                    f"(L:{cntB_left} C:{cntB_center} R:{cntB_right})")
                self.current_state = "AVOIDING"

            # Creep forward while turning – smoother arc than pure pivot
            self.current_twist.linear.x = 0.08
            diff = cntB_left - cntB_right
            if diff > 3:
                self.current_twist.angular.z = -0.9
                self.last_turn = -0.9
            elif diff < -3:
                self.current_twist.angular.z =  0.9
                self.last_turn =  0.9
            else:
                self.current_twist.angular.z = self.last_turn
            return

        # ── Priority 4: STRAIGHT – path is clear ─────────────────────────
        if self.current_state != "STRAIGHT":
            self.get_logger().info("Path clear → cruising forward.")
            self.current_state = "STRAIGHT"

        self.last_obstacle_time = 0.0          # reset stuck clock
        self.current_twist.linear.x  = 0.45   # moderate cruise speed
        self.current_twist.angular.z = 0.0

    # ────────────────────────────────────────────────────────────────────────
    def timer_callback(self):
        """Publish at 10 Hz regardless of map update rate."""
        self.cmd_pub.publish(self.current_twist)


# ────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = SimObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted – stopping robot.")
    finally:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
