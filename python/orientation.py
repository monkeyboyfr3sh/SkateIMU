import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

# === Geometry ===
cube_verts = np.array([
    [-1, -1, -1], [+1, -1, -1], [+1, +1, -1], [-1, +1, -1],
    [-1, -1, +1], [+1, -1, +1], [+1, +1, +1], [-1, +1, +1]
]) * 0.5

cube_edges = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7)
]

from imu_device import UARTIMUDevice  # if saved separately

device = UARTIMUDevice(port='COM7', baudrate=115200)

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

try:
    while True:
        quat = device.sample()
        if quat is None:
            continue

        rot = R.from_quat([quat[1], quat[2], quat[3], quat[0]])  # x, y, z, w
        rotated = rot.apply(cube_verts)

        ax.cla()
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        ax.set_title("ESP32 IMU Orientation")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        for start, end in cube_edges:
            ax.plot(
                [rotated[start][0], rotated[end][0]],
                [rotated[start][1], rotated[end][1]],
                [rotated[start][2], rotated[end][2]],
                color='blue'
            )

        plt.pause(0.01)

except KeyboardInterrupt:
    print("Stopped.")

finally:
    device.close()
