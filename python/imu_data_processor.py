import numpy as np

class IMUDataProcessor:
    def __init__(self, accel_deadband=0.05, velocity_deadband=0.01, velocity_decay=0.2):
        self.prev_time = None
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)

        self.accel_deadband = accel_deadband
        self.velocity_deadband = velocity_deadband
        self.velocity_decay = velocity_decay

    def reset(self):
        self.prev_time = None
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)

    def apply_deadband(self, vec, threshold):
        output = np.zeros_like(vec)
        for i in range(3):
            if i == 0 or i == 1:
                continue
            if abs(vec[i]) > threshold:
                output[i] = vec[i]
        return output

    def process_sample(self, timestamp, accel_world):
        # Apply acceleration deadband
        accel_world = self.apply_deadband(accel_world, self.accel_deadband)

        if self.prev_time is None:
            self.prev_time = timestamp
            return accel_world, self.velocity.copy(), self.position.copy()

        dt = timestamp - self.prev_time
        self.prev_time = timestamp

        # Integrate to velocity and apply decay
        self.velocity += accel_world * dt
        self.velocity *= self.velocity_decay

        # Apply velocity deadband
        velocity_filtered = self.apply_deadband(self.velocity, self.velocity_deadband)

        # Integrate to position
        self.position += velocity_filtered * dt

        return accel_world, velocity_filtered.copy(), self.position.copy()
