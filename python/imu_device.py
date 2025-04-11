from abc import ABC, abstractmethod
import serial
import numpy as np
from scipy.spatial.transform import Rotation as R


# === Base Device Class ===
class IMUDevice(ABC):
    """
    Abstract base class for IMU devices.
    """

    @abstractmethod
    def sample(self):
        """
        Reads and returns a quaternion [w, x, y, z] from the device.
        Should return None if data is not valid or not available.
        """
        pass

    @abstractmethod
    def close(self):
        """
        Closes any underlying connections to the device.
        """
        pass


# === UART Device Implementation ===
class UARTIMUDevice(IMUDevice):
    def __init__(self, port, baudrate=115200, timeout=1.0):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    def sample(self):
        """
        Reads one line from the UART, parses a quaternion if valid.
        Returns [w, x, y, z] or None if invalid.
        """
        while True:
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line.startswith("LINEARACCEL"):
                    continue

                parts = line.split(',')
                if len(parts) != 8:
                    print(f"Malformed line: {line}")
                    return None

                _, _, _, _, qw, qx, qy, qz = parts
                quat = [float(qw), float(qx), float(qy), float(qz)]

                if np.isclose(np.linalg.norm(quat), 0.0):
                    print("Invalid quaternion, skipping")
                    return None

                return quat

            except Exception as e:
                print(f"UART Read Error: {e}")
                return None

    def close(self):
        self.ser.close()
