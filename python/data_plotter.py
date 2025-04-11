import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time
from collections import deque
from scipy.spatial.transform import Rotation as R

# === Config ===
SERIAL_PORT = 'COM7'  # Change to your ESP32 port
BAUD_RATE = 115200
DEADBAND_THRESHOLD = 0.1
VELOCITY_DAMPING_THRESHOLD = 0.02
ACCEL_ZERO_THRESHOLD = 0.05
VELOCITY_ZERO_HOLD_TIME = 0.5
PLOT_WINDOW_SECONDS = 30.0
PLOT_MOVEMENT_THRESHOLD = 0.001  # Only update plot if position changes more than this

# === Utility functions ===
def apply_deadband(accel, threshold):
    return np.where(np.abs(accel) < threshold, 0, accel)

def damp_velocity(vel, threshold):
    return np.where(np.abs(vel) < threshold, 0, vel)

def is_valid_quaternion(quat):
    return not np.isclose(np.linalg.norm(quat), 0.0)

# === Setup Serial and Plot ===
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
plt.ion()
fig = plt.figure()
plot_ax = fig.add_subplot(111, projection='3d')

positions = deque()
timestamps = deque()
velocity = np.zeros(3)
position = np.zeros(3)
last_plot_position = np.zeros(3)

last_time = time.time()
zero_accel_time = 0.0

# --- Bias Calibration ---
calibration_samples = []
print("Calibrating... keep the device still for 2 seconds")
start_time = time.time()
while time.time() - start_time < 2.0:
    line = ser.readline().decode().strip()
    if line.startswith("LINEARACCEL"):
        parts = line.split(',')
        if len(parts) == 8:
            _, x, y, z, *_ = parts
            calibration_samples.append(np.array([float(x), float(y), float(z)]))
accel_offset = np.mean(calibration_samples, axis=0)
print(f"Accel bias: {accel_offset}")

# --- Main Loop ---
while True:
    try:
        line = ser.readline().decode().strip()
        if not line.startswith("LINEARACCEL"):
            print(line)
            continue

        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        parts = line.split(',')
        if len(parts) != 8:
            print(f"Malformed line: {line}")
            continue

        _, ax, ay, az, qw, qx, qy, qz = parts
        accel_local = np.array([float(ax), float(ay), float(az)])
        quat = [float(qw), float(qx), float(qy), float(qz)]

        if not is_valid_quaternion(quat):
            print("Warning: Skipping zero quaternion.")
            continue

        accel_local -= accel_offset
        accel_local = apply_deadband(accel_local, DEADBAND_THRESHOLD)

        rotation = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
        accel_world = rotation.apply(accel_local)

        velocity += accel_world * dt

        if np.linalg.norm(accel_world) < ACCEL_ZERO_THRESHOLD:
            zero_accel_time += dt
            if zero_accel_time >= VELOCITY_ZERO_HOLD_TIME:
                velocity[:] = 0
        else:
            zero_accel_time = 0

        velocity = damp_velocity(velocity, VELOCITY_DAMPING_THRESHOLD)

        # Integrate velocity to position
        position += velocity * dt
        positions.append(position.copy())
        timestamps.append(current_time)

        # Trim old data outside plot window
        while timestamps and current_time - timestamps[0] > PLOT_WINDOW_SECONDS:
            timestamps.popleft()
            positions.popleft()

        print(f"dt: {dt:.3f}s, Accel(local): {accel_local}, Accel(world): {accel_world}, Vel: {velocity}, Pos: {position}")

        # Only update plot if we've moved enough
        if np.linalg.norm(position - last_plot_position) > PLOT_MOVEMENT_THRESHOLD:
            last_plot_position = position.copy()

            plot_ax.cla()
            pos_np = np.array(positions)
            plot_ax.plot(pos_np[:, 0], pos_np[:, 1], pos_np[:, 2], marker='o')
            plot_ax.set_title('Live 3D Position from BNO055 (World Frame)')
            plot_ax.set_xlabel('X')
            plot_ax.set_ylabel('Y')
            plot_ax.set_zlabel('Z')
            fig.canvas.draw()
            fig.canvas.flush_events()

    except KeyboardInterrupt:
        print("Stopping...")
        break
    except Exception as e:
        print(f"Error: {e}")
