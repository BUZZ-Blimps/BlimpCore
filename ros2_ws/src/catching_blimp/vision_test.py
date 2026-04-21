#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Matches `enum target_type` in `CatchingBlimp.hpp`.
TARGET_TYPE_BALL = 0
TARGET_TYPE_GOAL = 1
TARGET_TYPE_NO_TARGET = 2

# Matches thresholds in `CatchingBlimp.hpp`.
BALL_GATE_OPEN_TRIGGER = 2000.0
BALL_CATCH_TRIGGER = 12500.0
GOAL_SCORE_TRIGGER = 30000.0


class VisionTestPublisher(Node):
    """Interactive mock vision publisher for the catching blimp state machine."""

    def __init__(self):
        super().__init__("vision_test_publisher")
        # State machine subscribes to relative topic name "targets".
        # Keeping it relative preserves ROS namespace behavior (e.g. /blimp1/targets).
        self.publisher_ = self.create_publisher(Float64MultiArray, "targets", 10)
        self.running = True
        self.next_target_id = 1

    def publish_target(
        self,
        target_type: int,
        bbox_width: float,
        bbox_height: float,
        u: float = 320.0,
        v: float = 240.0,
        z_meters: float = 2.0,
        theta_x_rad: float = 0.0,
        theta_y_rad: float = 0.0,
    ) -> None:
        """
        Publish a single vision detection.

        Expected field order for `targets` in CatchingBlimp:
          [0] u pixel center x (>= 0 means valid detection)
          [1] v pixel center y
          [2] z/depth estimate
          [3] object track id
          [4] target type enum: 0=ball, 1=goal, 2=no_target
          [5] theta_x in radians (converted to degrees in state machine)
          [6] theta_y in radians
          [7] bbox width (pixels)
          [8] bbox height (pixels)
        """
        msg = Float64MultiArray()
        msg.data = [
            float(u),
            float(v),
            float(z_meters),
            float(self.next_target_id),
            float(target_type),
            float(theta_x_rad),
            float(theta_y_rad),
            float(bbox_width),
            float(bbox_height),
        ]
        self.publisher_.publish(msg)
        self.get_logger().info(
            "Published target id=%d type=%d bbox_area=%.1f",
            self.next_target_id,
            target_type,
            bbox_width * bbox_height,
        )
        self.next_target_id += 1

    def publish_no_detection(self) -> None:
        """Publish a no-detection frame by setting u to -1."""
        msg = Float64MultiArray()
        msg.data = [-1.0, 0.0, 0.0, -1.0, float(TARGET_TYPE_NO_TARGET), 0.0, 0.0, 0.0, 0.0]
        self.publisher_.publish(msg)
        self.get_logger().info("Published no-detection frame")


def ros_spin_worker(node: VisionTestPublisher) -> None:
    """Keep ROS callbacks alive while CLI waits on user input."""
    while rclpy.ok() and node.running:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.01)


def print_menu() -> None:
    print("\n=== Vision Test Menu ===")
    print("[f] Ball found (small box): transitions searching -> approach")
    print("[c] Ball close/catch (large box): triggers catch thresholds")
    print("[g] Goal found (large box): supports goalSearch -> approachGoal")
    print("[n] No detection frame: simulates target loss")
    print("[m] Show this menu")
    print("[q] Quit")


def main(args=None):
    rclpy.init(args=args)
    node = VisionTestPublisher()
    spin_thread = threading.Thread(target=ros_spin_worker, args=(node,), daemon=True)
    spin_thread.start()

    print_menu()
    try:
        while rclpy.ok() and node.running:
            cmd = input("\nvision_test> ").strip().lower()
            if cmd == "f":
                # area=1200 < BALL_GATE_OPEN_TRIGGER (2000.0): "found" but not gate-open/catch.
                node.publish_target(target_type=TARGET_TYPE_BALL, bbox_width=40.0, bbox_height=30.0)
            elif cmd == "c":
                # area=13200 > BALL_CATCH_TRIGGER (12500.0): should trigger catching transition.
                node.publish_target(target_type=TARGET_TYPE_BALL, bbox_width=120.0, bbox_height=110.0)
            elif cmd == "g":
                # area=35200 > GOAL_SCORE_TRIGGER (30000.0): should trigger scoring transition.
                node.publish_target(target_type=TARGET_TYPE_GOAL, bbox_width=220.0, bbox_height=160.0)
            elif cmd == "n":
                node.publish_no_detection()
            elif cmd == "m":
                print_menu()
            elif cmd == "q":
                node.running = False
            elif cmd:
                print(f"Unknown command: {cmd}")
                print_menu()
    except (KeyboardInterrupt, EOFError):
        node.running = False
    finally:
        node.running = False
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()