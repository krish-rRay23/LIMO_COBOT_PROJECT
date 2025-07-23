import socket, threading, time

found_ip = None
def try_ip(ip):
    global found_ip
    if found_ip: return
    try:
        with socket.create_connection((ip, 9000), timeout=0.2):
            found_ip = ip
            print(f"✅ Found MyCobot at {ip}")
    except: pass

def scan(subnet="192.168.137.", timeout=3):
    print("🔍 Scanning for MyCobot...")
    threads = [threading.Thread(target=try_ip, args=(f"{subnet}{i}",)) for i in range(1, 255)]
    [t.start() for t in threads]
    start = time.time()
    while not found_ip and time.time() - start < timeout:
        time.sleep(0.05)
    [t.join(0.05) for t in threads]
    return found_ip or "❌ Not found"

if __name__ == "__main__":
    print(f"\n🔎 Result: {scan()}")
