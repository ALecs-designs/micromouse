import time
import board
import adafruit_vl53l0x
import digitalio
from .hardware_setup import i2c

# XSHUT pin mapping for sensors (BCM)
xshut_pins = [board.D27, board.D22, board.D17]  # left, front, right
target_addresses = [0x30, 0x31, 0x32]           # left, front, right

def scan_hex():
    return [hex(x) for x in i2c.scan()]

def map_and_initialize_vl53(retries_per_sensor=5, boot_delay=0.20, address_commit_delay=0.10):
    xshuts = []
    sensors = []

    # Prepare XSHUTs and hold all sensors in reset (LOW)
    for pin in xshut_pins:
        p = digitalio.DigitalInOut(pin)
        p.direction = digitalio.Direction.OUTPUT
        p.value = False
        xshuts.append(p)
    time.sleep(0.10)

    # Optional: mapping (single-enable scan)
    print("\n=== Per-sensor single-enable scan ===\n")
    for i, pin in enumerate(xshut_pins):
        for p in xshuts:
            p.value = False
        time.sleep(0.02)
        xshuts[i].value = True
        time.sleep(boot_delay)
        print(f"S{i+1} XSHUT={pin} bus:", scan_hex())

    # Sequential addressing with retries and verification
    print("\n=== Sequential addressing (FIXED) ===\n")
    for i, addr in enumerate(target_addresses):
        print(f"\nAddressing S{i+1} -> {hex(addr)}")

        # Keep previously addressed sensors ON (0..i), keep future sensors OFF (i+1..)
        # This prevents already-addressed sensors from resetting back to 0x29.
        for j, p in enumerate(xshuts):
            p.value = (j <= i)
        time.sleep(boot_delay)

        success = False
        for attempt in range(1, retries_per_sensor + 1):
            before = i2c.scan()
            print(f"[S{i+1} try {attempt}] before: {[hex(d) for d in before]}")

            # At this moment, ONLY the current sensor should be at 0x29
            if 0x29 not in before:
                print("⚠ 0x29 not present; toggling current sensor and retrying")
                xshuts[i].value = False
                time.sleep(0.05)
                xshuts[i].value = True
                time.sleep(boot_delay)
                continue

            try:
                # Create driver at default address (0x29)
                vl = adafruit_vl53l0x.VL53L0X(i2c)
                time.sleep(0.02)

                # Change its address
                vl.set_address(addr)
                time.sleep(address_commit_delay)

                after = i2c.scan()
                print(f"           after: {[hex(d) for d in after]}")

                if addr in after:
                    sensors.append(vl)
                    print(f"✅ S{i+1} now at {hex(addr)}")
                    success = True
                    break
                else:
                    print("⚠ Address not visible after set_address; retrying…")

            except Exception as e:
                print(f"⚠ Exception while addressing S{i+1}: {e}")

            # Retry: toggle ONLY this sensor (do NOT toggle earlier sensors)
            xshuts[i].value = False
            time.sleep(0.05)
            xshuts[i].value = True
            time.sleep(boot_delay)

        if not success:
            print(f"🚫 Sensor S{i+1} failed after {retries_per_sensor} attempts")
            return None

    # Turn all sensors ON (do NOT reset them)
    for p in xshuts:
        p.value = True
    time.sleep(0.10)

    print("\nFinal bus:", scan_hex())

    # Optional: verify all target addresses present
    final = set(i2c.scan())
    for addr in target_addresses:
        if addr not in final:
            print(f"❌ Missing {hex(addr)} on final scan — sensor reset/glitched")
            return None
    return sensors

# sensor binarization function definition
def binarize_sensors(distances, threshold=120): #Converts raw mm distances to True/False walls.
    
    walls = {'L': False, 'F': False, 'R': False}
    if distances[0] is not None and distances[0] < threshold: walls['L'] = True
    if distances[1] is not None and distances[1] < threshold: walls['F'] = True
    if distances[2] is not None and distances[2] < threshold: walls['R'] = True
    return walls

def sensor_mapping(directions_list, heading): # return array
    # f_dir = directions[heading]
    # r_dir = directions[(heading + 1) % 4]
    # l_dir = directions[(heading - 1) % 4]
    
    front = directions_list[heading]
    right = directions_list [(heading+1) % 4]
    left = directions_list [(heading-1) % 4]

    return front, right, left