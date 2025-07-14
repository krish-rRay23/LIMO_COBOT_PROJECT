from pymycobot.mycobot import MyCobot
import time

mc = MyCobot("/dev/ttyUSB0", 115200)
mc.power_on()
time.sleep(1)

# Move to known position
mc.send_angles([0, 30, -30, 0, 0, 0], 50)
time.sleep(3)

# Optional: Get feedback
angles = mc.get_angles()
print("Current Angles:", angles)
