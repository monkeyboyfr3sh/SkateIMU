from abc import ABC, abstractmethod
import serial
import numpy as np
from scipy.spatial.transform import Rotation as R


# === Base Device Class ===
class IMUDevice(ABC):
    @abstractmethod
    def sample(self):
        """
        Reads and returns (timestamp, linear_accel_world, orientation_quat)
        - timestamp: float (seconds)
        - linear_accel_world: np.array([ax, ay, az]) in world frame
        - orientation_quat: [w, x, y, z]
        """
        pass

    @abstractmethod
    def close(self):
        pass


# === UART Device Implementation ===
class UARTIMUDevice(IMUDevice):
    def __init__(self, port, baudrate=115200, timeout=1.0):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    def sample(self):
        """
        Parses lines like:
        I (82763) MAIN: PROC,<timestamp>,<ax>,<ay>,<az>,<vx>,<vy>,<vz>,<px>,<py>,<pz>,<qw>,<qx>,<qy>,<qz>
        Returns:
            timestamp (sec), accel (np.array), velocity (np.array), position (np.array), quat ([w,x,y,z]), raw_line (str)
        """
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return None, None, None, None, None, line

            if "PROC," not in line:
                return None, None, None, None, None, line

            # Strip log prefix (e.g., "I (82763) MAIN: ")
            proc_start = line.find("PROC,")
            proc_line = line[proc_start:]

            parts = proc_line.split(',')
            if len(parts) != 15:
                return None, None, None, None, None, line

            _, ts_str, ax, ay, az, vx, vy, vz, px, py, pz, qw, qx, qy, qz = parts
            timestamp = int(ts_str) / 1e6  # µs to seconds

            accel = np.array([float(ax), float(ay), float(az)])
            velocity = np.array([float(vx), float(vy), float(vz)])
            position = np.array([float(px), float(py), float(pz)])
            quat = [float(qw), float(qx), float(qy), float(qz)]

            return timestamp, accel, velocity, position, quat, line

        except Exception as e:
            return None, None, None, None, None, f"ERROR: {e}"

    def close(self):
        self.ser.close()
