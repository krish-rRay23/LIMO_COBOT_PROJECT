from pymycobot import MyCobot280Socket
import time

mc = MyCobot280Socket("192.168.137.79", 9000)

# Pose definitions
center_pose = [-0.03, -0.03, -0.03, -0.03, -0.03, -0.03]
pickup_pose = [-37.7, 113.11, 20.74, -104.94, 67.93, 10.63]

# Step 0: Open gripper
mc.set_gripper_state(0, 40)
print("🔓 Gripper OPENED")
time.sleep(1.5)

# Step 1: Move to center
mc.send_angles(center_pose, 40)
print("🎯 Moving to CENTER...")
time.sleep(2)

# Step 2: Go to pickup
mc.send_angles(pickup_pose, 40)
print("🤖 Moving to PICKUP...")
time.sleep(2.5)

# Step 3: Grab
mc.set_gripper_state(1, 40)
print("🧲 Gripper ON — Object grabbed")
time.sleep(2)

# Step 4: Return to center
mc.send_angles(center_pose, 40)
print("↩️ Returning to CENTER with object...")
time.sleep(2.5)

# Step 5: Wait
print("⏱️ Waiting for 5 seconds...")
time.sleep(5)

# Step 6: Go back to pickup pose
mc.send_angles(pickup_pose, 40)
print("🔁 Going back to PICKUP for drop...")
time.sleep(2.5)

# Step 7: Drop object
mc.set_gripper_state(0, 40)
print("🪣 Dropped object (gripper OFF)")
time.sleep(2)

# Step 8: Return to center again
mc.send_angles(center_pose, 40)
print("🎯 Final RETURN to CENTER")
time.sleep(2.5)

print("✅ Full cycle completed.")
