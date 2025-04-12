import numpy as np
from PyQt5 import QtCore

class IMUDataProcessor:
    def __init__(self, accel_deadband=0.05, velocity_decay=0.2, deadband_release_window=0.3):
        self.prev_time = None
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)

        self.accel_deadband = accel_deadband
        self.velocity_decay = velocity_decay
        self._deadband_release_window = deadband_release_window
        self._last_accel_active_time = QtCore.QTime.currentTime().msecsSinceStartOfDay() / 1000.0

    def reset(self):
        self.prev_time = None
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)

    def apply_deadband(self, accel):
        now = QtCore.QTime.currentTime().msecsSinceStartOfDay() / 1000.0
        if np.any(np.abs(accel) > self.accel_deadband):
            self._last_accel_active_time = now

        if now - self._last_accel_active_time < self._deadband_release_window:
            return accel
        return np.zeros_like(accel)

    def process_sample(self, timestamp, accel_world):
        accel_world = self.apply_deadband(accel_world)

        if self.prev_time is None:
            self.prev_time = timestamp
            return accel_world, self.velocity.copy(), self.position.copy()

        dt = timestamp - self.prev_time
        self.prev_time = timestamp

        self.velocity += accel_world * dt
        self.velocity *= self.velocity_decay
        self.position += self.velocity * dt

        return accel_world, self.velocity.copy(), self.position.copy()
