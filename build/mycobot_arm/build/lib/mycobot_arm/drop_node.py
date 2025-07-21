from pymycobot import MyCobot280Socket
import time
import rclpy
from rclpy.node import Node
import subprocess
import shlex

class DropNode(Node):
    def __init__(self):
        super().__init__('drop_node')
        self.declare_parameter("m5_ip", "192.168.137.75")  # default fallback
        ip = self.get_parameter("m5_ip").value
        self.mc = MyCobot280Socket(ip, 9000)
        self.get_logger().info("🤖 Connected to MyCobot for dropping")
        self.run_drop_sequence()

    def say(self, text):
        try:
            safe_text = shlex.quote(text)
            subprocess.call(f"espeak -s 135 -a 200 {safe_text} --stdout | aplay -D plughw:1,3", shell=True)
        except Exception as e:
            self.get_logger().warn(f"🔇 Voice error: {e}")

    def run_drop_sequence(self):
        center = [-0.03]*6
        pickup = [-37.7, 113.11, 20.74, -104.94, 67.93, 10.63]

        # Step 1: Move to center
        self.mc.send_angles(center, 40)
        self.get_logger().info("🎯 Moving to CENTER...")
        self.say("Stretching out... getting ready to drop!")
        time.sleep(2)

        # Step 2: Move to drop position
        self.mc.send_angles(pickup, 40)
        self.get_logger().info("🚶 Moving to DROP position...")
        self.say("Walking back to the drop zone. Here we go!")
        time.sleep(2.5)

        # Step 3: Release gripper
        self.mc.set_gripper_state(0, 40)
        self.get_logger().info("🪣 Dropped the object!")
        self.say("And... drop it like it's hot!")
        time.sleep(1.5)

        # Step 4: Return to center
        self.mc.send_angles(center, 40)
        self.get_logger().info("↩️ Back to CENTER after drop.")
        self.say("Done and dusted! I'm all cleaned up.")
        time.sleep(2.5)

        self.get_logger().info("✅ Drop sequence complete!")
        self.say("Mission complete! That was smooth.")

def main(args=None):
    rclpy.init(args=args)
    node = DropNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
