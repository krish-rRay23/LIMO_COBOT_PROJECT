from pymycobot import MyCobot280Socket
import time
import rclpy
from rclpy.node import Node
import subprocess
import shlex

class PickNode(Node):
    def __init__(self):
        super().__init__('pick_node')
        self.declare_parameter("m5_ip", "192.168.137.75")  # default fallback
        ip = self.get_parameter("m5_ip").value
        self.mc = MyCobot280Socket(ip, 9000)
        self.get_logger().info("🤖 Connected to MyCobot over Wi-Fi")

        self.run_pick_sequence()

    def say(self, text):
        try:
            safe_text = shlex.quote(text)
            subprocess.call(f"espeak -s 135 -a 200 {safe_text} --stdout | aplay -D plughw:1,3", shell=True)
        except Exception as e:
            self.get_logger().warn(f"🔇 Voice error: {e}")

    def run_pick_sequence(self):
        center = [-0.03]*6
        pickup = [-37.7, 113.11, 20.74, -104.94, 67.93, 10.63]

        # Step 0: Open gripper
        self.mc.set_gripper_state(0, 40)
        self.get_logger().info("👐 Gripper OPENED")
        self.say("Hello friend! I'm ready to grab something!")
        time.sleep(1)

        # Step 1: Move to center
        self.mc.send_angles(center, 40)
        self.get_logger().info("🎯 Moving to CENTER...")
        self.say("Let me stretch first!")
        time.sleep(1)

        # Step 2: Move to pickup
        self.mc.send_angles(pickup, 40)
        self.get_logger().info("🤖 Moving to PICKUP...")
        self.say("Approaching target. I'm going in!")
        time.sleep(1)

        # Step 3: Grab
        self.mc.set_gripper_state(1, 40)
        self.get_logger().info("🧲 Grabbed the object!")
        self.say("Yay! I caught it!")
        time.sleep(1)

        # Step 4: Return to center
        self.mc.send_angles(center, 40)
        self.get_logger().info("↩️ Returning to CENTER...")
        self.say("Back to base with my precious cargo!")
        time.sleep(1.5)

        self.get_logger().info("✅ Pick sequence complete!")
        #self.say("Mission complete! That was awesome!")

def main(args=None):
    rclpy.init(args=args)
    node = PickNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
