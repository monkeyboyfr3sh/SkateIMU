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
        Parses a line like: DATA,<timestamp>,<ax>,<ay>,<az>,<qw>,<qx>,<qy>,<qz>
        Rotates acceleration to world frame using quaternion.
        Returns: (timestamp_seconds, accel_world, [w, x, y, z], raw_line)
                or (None, None, None, raw_line) if parsing fails
        """
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return None, None, None, line

            if "DATA," not in line:
                return None, None, None, line

            parts = line.split(',')
            if len(parts) != 9:
                return None, None, None, line

            _, timestamp_str, ax, ay, az, qw, qx, qy, qz = parts
            timestamp = int(timestamp_str) / 1e6  # convert µs to seconds

            accel = np.array([float(ax), float(ay), float(az)])
            quat = [float(qw), float(qx), float(qy), float(qz)]

            # Convert quaternion to rotation object
            rotation = R.from_quat([qx, qy, qz, qw])  # scipy expects [x, y, z, w]
            accel_world = rotation.apply(accel)

            return timestamp, accel_world, quat, line

        except Exception as e:
            return None, None, None, f"ERROR: {e}"

    def close(self):
        self.ser.close()
