import sys
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from imu_device import UARTIMUDevice

# Load and scale mesh
mesh = trimesh.load('C:/Users/monke/Documents/GitHub/hello_world/python/assets/Skateboard - 172002/files/Skateboard_WHOLE.stl')
mesh.apply_scale(0.81 / mesh.extents[0])
mesh.vertices -= mesh.centroid
vertices = mesh.vertices.copy()
faces = mesh.faces
mesh_data = gl.MeshData(vertexes=vertices, faces=faces)

# Main application window
class IMUMeshViewer(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU Mesh + Live Plot")
        self.resize(1400, 800)

        # Layouts
        main_layout = QtWidgets.QHBoxLayout(self)
        left_layout = QtWidgets.QVBoxLayout()
        right_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(left_layout, 2)
        main_layout.addLayout(right_layout, 1)

        # 3D View
        self.view = gl.GLViewWidget()
        self.view.setWindowTitle('3D IMU Mesh View')
        self.view.addItem(gl.GLGridItem())
        self.mesh_item = gl.GLMeshItem(meshdata=mesh_data, smooth=False, drawFaces=True, drawEdges=True, edgeColor=(0, 0, 0, 1), color=(0.6, 0.8, 1, 1))
        self.mesh_item.setGLOptions('opaque')
        self.view.addItem(self.mesh_item)
        left_layout.addWidget(self.view)

        # Quaternion plot
        self.quat_plot = pg.PlotWidget(title="Quaternion (w, x, y, z)")
        self.quat_plot.addLegend()
        self.quat_plot.setYRange(-1.2, 1.2)
        self.quat_curves = [self.quat_plot.plot(pen=pg.mkPen(c, width=2), name=label)
                            for c, label in zip(['w', 'r', 'g', 'b'], ['w', 'x', 'y', 'z'])]
        right_layout.addWidget(self.quat_plot)

        # Acceleration plot
        self.accel_plot = pg.PlotWidget(title="Linear Acceleration (x, y, z)")
        self.accel_plot.addLegend()
        self.accel_plot.setYRange(-20, 20)
        self.accel_curves = [self.accel_plot.plot(pen=pg.mkPen(c, width=2), name=label)
                             for c, label in zip(['r', 'g', 'b'], ['x', 'y', 'z'])]
        right_layout.addWidget(self.accel_plot)

        # IMU Device
        self.device = UARTIMUDevice(port='COM7', baudrate=115200)

        # State
        self.prev_time = None
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)
        self.timestamps = []
        self.quat_data = [[] for _ in range(4)]
        self.accel_data = [[] for _ in range(3)]
        self.max_points = 300

        # Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(33)

    def update_data(self):
        result = self.device.sample()
        if result is None:
            return

        timestamp, accel_world, quat = result

        if self.prev_time is None:
            self.prev_time = timestamp
            return

        dt = timestamp - self.prev_time
        self.prev_time = timestamp

        # Integrate acceleration -> velocity -> position
        self.velocity += accel_world * dt
        self.position += self.velocity * dt

        # Rotate mesh
        rot = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix()
        transformed_vertices = (rot @ vertices.T).T + self.position  # <-- apply translation here
        mesh_data.setVertexes(transformed_vertices)
        self.mesh_item.meshDataChanged()

        # Update plots
        self.timestamps.append(timestamp)
        for i in range(4):
            self.quat_data[i].append(quat[i])
        for i in range(3):
            self.accel_data[i].append(accel_world[i])

        if len(self.timestamps) > self.max_points:
            self.timestamps = self.timestamps[-self.max_points:]
            for i in range(4):
                self.quat_data[i] = self.quat_data[i][-self.max_points:]
            for i in range(3):
                self.accel_data[i] = self.accel_data[i][-self.max_points:]

        for i in range(4):
            self.quat_curves[i].setData(self.timestamps, self.quat_data[i])
        for i in range(3):
            self.accel_curves[i].setData(self.timestamps, self.accel_data[i])


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = IMUMeshViewer()
    window.show()
    sys.exit(app.exec_())
