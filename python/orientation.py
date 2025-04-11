import sys
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from imu_device import UARTIMUDevice

class IMUSampleThread(QtCore.QThread):
    new_sample = QtCore.pyqtSignal(float, np.ndarray, list)  # timestamp, accel_world, quat

    def __init__(self, device):
        super().__init__()
        self.device = device
        self.running = True

    def run(self):
        while self.running:
            result = self.device.sample()
            if result:
                timestamp, accel_world, quat = result
                self.new_sample.emit(timestamp, accel_world, quat)
            self.msleep(10)

    def stop(self):
        self.running = False
        self.wait()

class IMUMeshViewer(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU Mesh + Live Plot")
        self.resize(1400, 800)

        # Load and scale mesh
        mesh = trimesh.load('C:/Users/monke/Documents/GitHub/hello_world/python/assets/Skateboard - 172002/files/Skateboard_WHOLE.stl')
        mesh.apply_scale(0.81 / mesh.extents[0])
        mesh.vertices -= mesh.centroid
        self.vertices = mesh.vertices.copy()
        faces = mesh.faces
        self.mesh_data = gl.MeshData(vertexes=self.vertices, faces=faces)

        # Layout
        main_layout = QtWidgets.QHBoxLayout(self)
        left_layout = QtWidgets.QVBoxLayout()
        right_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(left_layout, 2)
        main_layout.addLayout(right_layout, 1)

        # 3D view
        self.view = gl.GLViewWidget()
        self.view.addItem(gl.GLGridItem())
        self.mesh_item = gl.GLMeshItem(meshdata=self.mesh_data, smooth=False, drawFaces=True, drawEdges=True, edgeColor=(0, 0, 0, 1), color=(0.6, 0.8, 1, 1))
        self.view.addItem(self.mesh_item)
        left_layout.addWidget(self.view)

        # Plot widgets
        self.quat_curves, self.accel_curves, self.velocity_curves, self.position_curves = [], [], [], []
        self.init_plot(right_layout)

        # IMU + threading
        self.device = UARTIMUDevice(port='COM7', baudrate=115200)
        self.sample_thread = IMUSampleThread(self.device)
        self.sample_thread.new_sample.connect(self.handle_sample)
        self.sample_thread.start()

        # State
        self.prev_time = None
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)
        self.timestamps = []
        self.quat_data = [[] for _ in range(4)]
        self.accel_data = [[] for _ in range(3)]
        self.velocity_data = [[] for _ in range(3)]
        self.position_data = [[] for _ in range(3)]
        self.max_points = 300
        self.accel_deadband = 0.6
        self.velocity_decay = 0.60

    def init_plot(self, layout):
        # Quaternion
        self.quat_plot = pg.PlotWidget(title="Quaternion (w, x, y, z)")
        self.quat_plot.setYRange(-1.2, 1.2)
        self.quat_plot.addLegend()
        self.quat_curves = [self.quat_plot.plot(pen=pg.mkPen(c, width=2), name=label)
                            for c, label in zip(['w', 'r', 'g', 'b'], ['w', 'x', 'y', 'z'])]
        layout.addWidget(self.quat_plot)

        # Accel
        self.accel_plot = pg.PlotWidget(title="Linear Acceleration (x, y, z)")
        self.accel_plot.setYRange(-20, 20)
        self.accel_plot.addLegend()
        self.accel_curves = [self.accel_plot.plot(pen=pg.mkPen(c, width=2), name=label)
                             for c, label in zip(['r', 'g', 'b'], ['x', 'y', 'z'])]
        layout.addWidget(self.accel_plot)

        # Velocity
        self.velocity_plot = pg.PlotWidget(title="Velocity (x, y, z)")
        self.velocity_plot.setYRange(-5, 5)
        self.velocity_plot.addLegend()
        self.velocity_curves = [self.velocity_plot.plot(pen=pg.mkPen(c, width=2), name=label)
                                for c, label in zip(['r', 'g', 'b'], ['vx', 'vy', 'vz'])]
        layout.addWidget(self.velocity_plot)

        # Position
        self.position_plot = pg.PlotWidget(title="Position (x, y, z)")
        self.position_plot.setYRange(-5, 5)
        self.position_plot.addLegend()
        self.position_curves = [self.position_plot.plot(pen=pg.mkPen(c, width=2), name=label)
                                for c, label in zip(['r', 'g', 'b'], ['px', 'py', 'pz'])]
        layout.addWidget(self.position_plot)

    def apply_deadband(self, accel):
        return np.where(np.abs(accel) < self.accel_deadband, 0, accel)

    def handle_sample(self, timestamp, accel_world, quat):
        accel_world = self.apply_deadband(accel_world)

        if self.prev_time is None:
            self.prev_time = timestamp
            return

        dt = timestamp - self.prev_time
        self.prev_time = timestamp

        self.velocity += accel_world * dt
        self.velocity *= self.velocity_decay
        self.position += self.velocity * dt

        # Transform mesh
        rot = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix()
        transformed_vertices = (rot @ self.vertices.T).T + self.position
        self.mesh_data.setVertexes(transformed_vertices)
        self.mesh_item.meshDataChanged()

        # Update plots
        self.timestamps.append(timestamp)
        for i in range(4):
            self.quat_data[i].append(quat[i])
        for i in range(3):
            self.accel_data[i].append(accel_world[i])
            self.velocity_data[i].append(self.velocity[i])
            self.position_data[i].append(self.position[i])

        if len(self.timestamps) > self.max_points:
            self.timestamps = self.timestamps[-self.max_points:]
            for d in (self.quat_data, self.accel_data, self.velocity_data, self.position_data):
                for i in range(len(d)):
                    d[i] = d[i][-self.max_points:]

        for i in range(4):
            self.quat_curves[i].setData(self.timestamps, self.quat_data[i])
        for i in range(3):
            self.accel_curves[i].setData(self.timestamps, self.accel_data[i])
            self.velocity_curves[i].setData(self.timestamps, self.velocity_data[i])
            self.position_curves[i].setData(self.timestamps, self.position_data[i])

    def closeEvent(self, event):
        self.sample_thread.stop()
        self.device.close()
        event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    viewer = IMUMeshViewer()
    viewer.show()
    sys.exit(app.exec_())
