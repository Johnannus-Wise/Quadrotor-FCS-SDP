import serial
import csv
import re
from datetime import datetime

# ==========================================
# Serial Configuration
# ==========================================
PORT = "COM4"
BAUDRATE = 921600
CSV_FILE = "flight_log.csv"

# ==========================================
# Regex Patterns
# ==========================================

sensor_pattern = re.compile(
    r"Pitch:\s*([-0-9.]+).*?"
    r"Roll:\s*([-0-9.]+).*?"
    r"Accel:\s*\(([-0-9.]+),\s*([-0-9.]+),\s*([-0-9.]+)\).*?"
    r"Gyro:\s*\(([-0-9.]+),\s*([-0-9.]+),\s*([-0-9.]+)\)"
)

altitude_pattern = re.compile(
    r"Altitude:\s*([-0-9.]+)\s*m,\s*"
    r"Pressure:\s*([-0-9.]+)\s*Pa,\s*"
    r"Temperature:\s*([-0-9.]+)"
)

pilot_pattern = re.compile(
    r"mainThrottleInput:\s*(-?\d+),\s*"
    r"Desired Pitch:\s*(-?\d+).*?,\s*"
    r"Desired Roll:\s*(-?\d+).*?,\s*"
    r"Yaw Speed:\s*(-?\d+).*?,\s*"
    r"Desired Altitude:\s*(-?\d+)"
)

pid_gains_pattern = re.compile(
    r"Roll Angle PID:\s*\[([-0-9., ]+)\],\s*"
    r"Pitch Angle PID:\s*\[([-0-9., ]+)\],\s*"
    r"Roll Rate PID:\s*\[([-0-9., ]+)\],\s*"
    r"Pitch Rate PID:\s*\[([-0-9., ]+)\],\s*"
    r"Yaw Rate PID:\s*\[([-0-9., ]+)\],\s*"
    r"Altitude PID:\s*\[([-0-9., ]+)\]"
)

pid_output_pattern = re.compile(
    r"PID Outputs.*?"
    r"Roll Angle:\s*([-0-9.]+),\s*"
    r"Pitch Angle:\s*([-0-9.]+),\s*"
    r"Roll Rate:\s*([-0-9.]+),\s*"
    r"Pitch Rate:\s*([-0-9.]+),\s*"
    r"Yaw Rate:\s*([-0-9.]+),\s*"
    r"Altitude:\s*([-0-9.]+)"
)

mode_pattern = re.compile(
    r"Altitude Lock:\s*(\d+|true|false),\s*"
    r"Angle Mode:\s*(\d+|true|false)"
)

motor_pattern = re.compile(
    r"motor outputs:\s*FL\s*(-?\d+),\s*"
    r"FR\s*(-?\d+),\s*"
    r"RR\s*(-?\d+),\s*"
    r"RL\s*(-?\d+),\s*"
    r"Reference Pitch:\s*(-?\d+).*?,\s*"
    r"Reference Roll:\s*(-?\d+)"
)

# ==========================================
# CSV Header
# ==========================================

header = [
    "Timestamp",

    "Pitch", "Roll",
    "AccelX", "AccelY", "AccelZ",
    "GyroX", "GyroY", "GyroZ",

    "Altitude", "Pressure", "Temperature",

    "ThrottleInput",
    "DesiredPitch",
    "DesiredRoll",
    "YawSpeed",
    "DesiredAltitude",

    "RollAngle_Kp", "RollAngle_Ki", "RollAngle_Kd",
    "PitchAngle_Kp", "PitchAngle_Ki", "PitchAngle_Kd",
    "RollRate_Kp", "RollRate_Ki", "RollRate_Kd",
    "PitchRate_Kp", "PitchRate_Ki", "PitchRate_Kd",
    "YawRate_Kp", "YawRate_Ki", "YawRate_Kd",
    "Altitude_Kp", "Altitude_Ki", "Altitude_Kd",

    "RollAngle_Output",
    "PitchAngle_Output",
    "RollRate_Output",
    "PitchRate_Output",
    "YawRate_Output",
    "Altitude_Output",

    "AltitudeLock",
    "AngleMode",

    "Motor_FL",
    "Motor_FR",
    "Motor_RR",
    "Motor_RL",

    "ReferencePitch",
    "ReferenceRoll"
]

# ==========================================
# Open Serial Port
# ==========================================

ser = serial.Serial(PORT, BAUDRATE, timeout=1)

print(f"Connected to {PORT} @ {BAUDRATE}")

with open(CSV_FILE, "w", newline="") as csvfile:

    writer = csv.DictWriter(csvfile, fieldnames=header)
    writer.writeheader()

    sample = {}

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if not line:
                continue

            print(line)

            m = sensor_pattern.search(line)
            if m:
                sample["Timestamp"] = datetime.now().isoformat()

                sample["Pitch"] = float(m.group(1))
                sample["Roll"] = float(m.group(2))

                sample["AccelX"] = float(m.group(3))
                sample["AccelY"] = float(m.group(4))
                sample["AccelZ"] = float(m.group(5))

                sample["GyroX"] = float(m.group(6))
                sample["GyroY"] = float(m.group(7))
                sample["GyroZ"] = float(m.group(8))
                continue

            m = altitude_pattern.search(line)
            if m:
                sample["Altitude"] = float(m.group(1))
                sample["Pressure"] = float(m.group(2))
                sample["Temperature"] = float(m.group(3))
                continue

            m = pilot_pattern.search(line)
            if m:
                sample["ThrottleInput"] = int(m.group(1))
                sample["DesiredPitch"] = int(m.group(2))
                sample["DesiredRoll"] = int(m.group(3))
                sample["YawSpeed"] = int(m.group(4))
                sample["DesiredAltitude"] = int(m.group(5))
                continue

            m = pid_gains_pattern.search(line)
            if m:

                names = [
                    "RollAngle",
                    "PitchAngle",
                    "RollRate",
                    "PitchRate",
                    "YawRate",
                    "Altitude"
                ]

                for i, name in enumerate(names):

                    kp, ki, kd = [
                        float(x.strip())
                        for x in m.group(i + 1).split(",")
                    ]

                    sample[f"{name}_Kp"] = kp
                    sample[f"{name}_Ki"] = ki
                    sample[f"{name}_Kd"] = kd

                continue

            m = pid_output_pattern.search(line)
            if m:
                sample["RollAngle_Output"] = float(m.group(1))
                sample["PitchAngle_Output"] = float(m.group(2))
                sample["RollRate_Output"] = float(m.group(3))
                sample["PitchRate_Output"] = float(m.group(4))
                sample["YawRate_Output"] = float(m.group(5))
                sample["Altitude_Output"] = float(m.group(6))
                continue

            m = mode_pattern.search(line)
            if m:
                sample["AltitudeLock"] = m.group(1)
                sample["AngleMode"] = m.group(2)
                continue

            m = motor_pattern.search(line)
            if m:
                sample["Motor_FL"] = int(m.group(1))
                sample["Motor_FR"] = int(m.group(2))
                sample["Motor_RR"] = int(m.group(3))
                sample["Motor_RL"] = int(m.group(4))

                sample["ReferencePitch"] = int(m.group(5))
                sample["ReferenceRoll"] = int(m.group(6))

                # Final line of a complete flightLog() cycle
                writer.writerow(sample)
                csvfile.flush()

                sample = {}

        except KeyboardInterrupt:
            print("\nLogging stopped.")
            break

        except Exception as e:
            print(f"Parse error: {e}")