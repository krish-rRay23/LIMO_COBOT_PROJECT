from pymycobot import MyCobot280Socket
import time
import subprocess
import shlex
import socket
import threading
import psutil

# 🔊 Voice function
def say(text):
    try:
        safe_text = shlex.quote(text)
        subprocess.call(f"espeak -s 135 -a 200 {safe_text} --stdout | aplay -D plughw:1,3", shell=True)
    except Exception as e:
        print(f"🔇 Voice error: {e}")

# 🔍 Auto-subnet detection
def get_subnet():
    interfaces = psutil.net_if_addrs()
    for iface_info in interfaces.values():
        for addr in iface_info:
            if addr.family == socket.AF_INET and not addr.address.startswith("127."):
                return ".".join(addr.address.split('.')[:3]) + '.'
    return "192.168.137."

# ⚡ Stable multithreaded scanner
def scan_for_mycobot(port=9000, timeout=0.3, max_threads=200):
    found_ip = None
    stop_event = threading.Event()
    lock = threading.Lock()

    def try_connect(ip):
        nonlocal found_ip
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(timeout)
                s.connect((ip, port))
                with lock:
                    if not found_ip:
                        found_ip = ip
                        stop_event.set()
                        print(f"✅ Found MyCobot at {ip}")
        except:
            pass

    base_ip = get_subnet()
    print(f"🔍 Scanning subnet: {base_ip}0/24...")
    say("Scanning network for MyCobot")
    threads = []
    for i in range(1, 255):
        if stop_event.is_set():
            break
        ip = f"{base_ip}{i}"
        t = threading.Thread(target=try_connect, args=(ip,))
        threads.append(t)
        t.start()

        while threading.active_count() > max_threads:
            time.sleep(0.01)

    for t in threads:
        t.join(timeout=timeout + 0.1)

    if not found_ip:
        print("❌ MyCobot not found.")
        say("MyCobot not found. Aborting.")
    return found_ip

# 🚀 Main sequence
ip = scan_for_mycobot()
if not ip:
    exit(1)

mc = MyCobot280Socket(ip, 9000)
print(f"🔗 Connected to MyCobot at {ip}:9000")
say("Connected to MyCobot. Let's begin the mission!")

# Pose definitions
center_pose = [-0.03] * 6
pickup_pose = [-37.7, 113.11, 20.74, -104.94, 67.93, 10.63]

# Step 0: Open gripper
mc.set_gripper_state(0, 40)
print("🔓 Gripper OPENED")
say("Gripper opened. Ready to pick.")
time.sleep(1.5)

# Step 1: Move to center
mc.send_angles(center_pose, 40)
print("🎯 Moving to CENTER...")
say("Heading to center position.")
time.sleep(2)

# Step 2: Go to pickup
mc.send_angles(pickup_pose, 40)
print("🤖 Moving to PICKUP...")
say("Approaching target.")
time.sleep(2.5)

# Step 3: Grab
mc.set_gripper_state(1, 40)
print("🧲 Gripper ON — Object grabbed")
say("Object secured!")
time.sleep(2)

# Step 4: Return to center
mc.send_angles(center_pose, 40)
print("↩️ Returning to CENTER with object...")
say("Returning with the item.")
time.sleep(2.5)

# Step 5: Wait
print("⏱️ Waiting for 5 seconds...")
say("Holding position.")
time.sleep(5)

# Step 6: Go back to pickup pose
mc.send_angles(pickup_pose, 40)
print("🔁 Going back to PICKUP for drop...")
say("Going back to drop point.")
time.sleep(2.5)

# Step 7: Drop object
mc.set_gripper_state(0, 40)
print("🪣 Dropped object (gripper OFF)")
say("Object dropped.")
time.sleep(2)

# Step 8: Return to center again
mc.send_angles(center_pose, 40)
print("🎯 Final RETURN to CENTER")
say("Final return to center complete.")
time.sleep(2.5)

print("✅ Full cycle completed.")
say("Mission accomplished. Good job, commander.")
