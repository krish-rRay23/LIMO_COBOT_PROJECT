import math


class ExplorationHandlerMock:
    def __init__(self):
        self.goal_sent = False
        self.exploring = True
        self.object_found = False
        self.current_index = 0
        self._goal_handle = None

        self.waypoints = [
            (2.3, -9.0, 90.0),
            (2.8, -8.5, 90.0),
            (3.2, -9.8, 0.0),
            (1.6, -10.3, 180.0),
            (1.2, -9.0, 270.0),
        ]

        print("🧪 [INIT] Mock ExplorationHandler ready.")

    def mock_main_loop(self):
        if not self.exploring or self.goal_sent:
            print("🚫 Skipping loop (exploring:", self.exploring, "goal_sent:", self.goal_sent, ")")
            return

        if self.current_index >= len(self.waypoints):
            print("✅ All waypoints completed.")
            self.exploring = False
            return

        x, y, yaw_deg = self.waypoints[self.current_index]
        yaw_rad = math.radians(yaw_deg)

        orientation_z = math.sin(yaw_rad / 2.0)
        orientation_w = math.cos(yaw_rad / 2.0)

        print(f"🚀 Sending waypoint {self.current_index + 1}: (x={x:.2f}, y={y:.2f}, yaw={yaw_deg}°)")
        print(f"🌀 Orientation (z={orientation_z:.3f}, w={orientation_w:.3f})")

        self.goal_sent = True
        self.goal_response_callback(accepted=True)

    def yolo_callback(self, msg_str):
        if self.object_found:
            print("🛑 YOLO callback ignored — object already found.")
            return

        print("⚠️  YOLO object detected.")
        self.object_found = True
        self.exploring = False

        if self.goal_sent:
            print("🛑 Cancelling active goal...")
            self.goal_sent = False

        try:
            label, cx, cy, coord_str = msg_str.split(':')
            x, y, z = map(float, coord_str.split(','))

            print(f"🎯 Navigating to object at: ({x:.2f}, {y:.2f}, {z:.2f})")
            self.goal_sent = True
            self.goal_response_callback(accepted=True)

        except Exception as e:
            print(f"❌ YOLO callback error: {e}")

    def goal_response_callback(self, accepted):
        if not accepted:
            print("❌ Navigation goal rejected.")
            self.goal_sent = False
            return

        print("📍 Goal accepted.")
        self.result_callback(success=True)

    def result_callback(self, success):
        if success:
            print("🏁 Goal reached.")
        else:
            print("⚠️ Goal failed.")

        if self.object_found:
            print("⏸ Exploration paused. Awaiting next action.")
        else:
            self.current_index += 1

        self.goal_sent = False


# ✅ Run test
if __name__ == '__main__':
    node = ExplorationHandlerMock()

    print("\n--- Simulating waypoint navigation loop ---")
    for _ in range(6):  # intentionally go beyond 5
        node.mock_main_loop()

    print("\n--- Simulating YOLO object detection ---")
    yolo_test_msg = "bottle:0.5:0.5:2.5,3.0,0.0"
    node.yolo_callback(yolo_test_msg)

    print("\n--- Try triggering YOLO again (should be ignored) ---")
    node.yolo_callback(yolo_test_msg)
