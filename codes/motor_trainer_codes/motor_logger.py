import serial
import os
import time

# ===== USER SETTINGS =====
PORT = "COM5"
BAUD = 115200
OUTPUT_DIR = "motor_logs_v2"

FILENAME = "Motorbad"   # change if needed
# =========================

os.makedirs(OUTPUT_DIR, exist_ok=True)

ser = serial.Serial(PORT, BAUD, timeout=1)

recording = False
csv_file = None

print("Connected to", PORT)
print("Waiting for ESP32 motor test...")

def generate_filename():
    timestamp = int(time.time() * 1000)
    return f"{FILENAME}.{timestamp}.csv"

while True:
    try:
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        print(line)

        # ===== START RECORDING =====
        if line == "timestamp,U1,V1,W1":
            filename = generate_filename()
            path = os.path.join(OUTPUT_DIR, filename)

            csv_file = open(path, "w", newline="")
            csv_file.write(line + "\n")
            recording = True

            print(f"\n🟢 Recording started -> {filename}\n")

        # ===== STOP RECORDING =====
        elif line == "Done":
            if recording and csv_file:
                csv_file.close()
                print("\n🔴 Recording saved\n")

            recording = False
            csv_file = None

        # ===== SAVE DATA =====
        elif recording and line[0].isdigit():
            csv_file.write(line + "\n")

    except KeyboardInterrupt:
        print("\nExiting...")
        break

    except Exception as e:
        print("Error:", e)
