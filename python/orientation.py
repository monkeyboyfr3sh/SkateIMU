import sys
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtGui import QVector3D
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from imu_device import UARTIMUDevice
from imu_data_processor import IMUDataProcessor

class IMUSampleThread(QtCore.QThread):
    new_sample = QtCore.pyqtSignal(float, np.ndarray, list, str)

    def __init__(self, device):
        super().__init__()
        self.device = device
        self.running = True

    def run(self):
        while self.running:
            timestamp, accel_world, quat, raw_line = self.device.sample()

            # Always emit a line, even if the parsed data is invalid
            if raw_line:
                # Use dummy values for signal consistency
                if timestamp is None:
                    timestamp = 0.0
                    accel_world = np.zeros(3)
                    quat = [1.0, 0.0, 0.0, 0.0]
                self.new_sample.emit(timestamp, accel_world, quat, raw_line)

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
        mesh = trimesh.load('C:/Users/monke/Documents/GitHub/SkateIMU/python/assets/Skateboard - 172002/files/Skateboard_WHOLE.stl')
        mesh.apply_scale(0.2757 / mesh.extents[0])
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
        # Set initial camera distance and center
        model_radius = np.linalg.norm(self.vertices, axis=1).max()
        initial_distance = model_radius * 3  # zoom factor, adjust as needed

        self.view.setCameraPosition(
            pos=QtGui.QVector3D(0, 0, initial_distance),
            distance=initial_distance,
            elevation=20,
            azimuth=45
        )
        left_layout.addWidget(self.view)

        # Plot widgets
        self.quat_curves, self.accel_curves, self.velocity_curves, self.position_curves = [], [], [], []
        self.init_plot(right_layout)

        # IMU + threading
        self.device = UARTIMUDevice(port='COM7', baudrate=115200)
        self.sample_thread = IMUSampleThread(self.device)
        self.sample_thread.new_sample.connect(self.handle_sample)
        self.sample_thread.start()
        self.processor = IMUDataProcessor(
            accel_deadband = 0.2,
            velocity_deadband = 0.05,
            velocity_decay = 0.90,
        )

        # State
        self.prev_time = None
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)
        self.timestamps = []
        self.quat_data = [[] for _ in range(4)]
        self.accel_data = [[] for _ in range(3)]
        self.velocity_data = [[] for _ in range(3)]
        self.position_data = [[] for _ in range(3)]
        self.max_points = 150

        # --- Add Reset Button
        self.reset_button = QtWidgets.QPushButton("Reset Position")
        self.reset_button.clicked.connect(self.reset_position)
        right_layout.addWidget(self.reset_button)

        self.terminal = QtWidgets.QTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setFixedHeight(150)
        right_layout.addWidget(self.terminal)

    def reset_position(self):
        self.processor.reset()
        for data in (self.position_data, self.velocity_data):
            for i in range(3):
                data[i].clear()
        for i in range(3):
            self.position_data[i].append(0.0)
            self.velocity_data[i].append(0.0)
        self.prev_time = None
        print("Position and velocity reset.")

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

    def handle_sample(self, timestamp, accel_world, quat, raw_line):

        # Always show the terminal line
        self.terminal.append(raw_line)
        self.terminal.verticalScrollBar().setValue(self.terminal.verticalScrollBar().maximum())
    
        # Only process if accel/quat data is valid
        if accel_world is None or quat is None:
            return

        # Process the sample using the processor
        accel_world, velocity, position = self.processor.process_sample(timestamp, accel_world)

        # Transform mesh using the position returned from processor
        rot = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix()
        transformed_vertices = (rot @ self.vertices.T).T + position
        self.mesh_data.setVertexes(transformed_vertices)
        self.mesh_item.meshDataChanged()

        # Make the camera follow the model
        self.view.opts['center'] = QVector3D(*position)

        # Append new data to logs
        self.timestamps.append(timestamp)
        for i in range(4):
            self.quat_data[i].append(quat[i])
        for i in range(3):
            self.accel_data[i].append(accel_world[i])
            self.velocity_data[i].append(velocity[i])
            self.position_data[i].append(position[i])

        # Trim data arrays if too long
        if len(self.timestamps) > self.max_points:
            trim = -self.max_points
            self.timestamps = self.timestamps[trim:]
            self.quat_data = [d[trim:] for d in self.quat_data]
            self.accel_data = [d[trim:] for d in self.accel_data]
            self.velocity_data = [d[trim:] for d in self.velocity_data]
            self.position_data = [d[trim:] for d in self.position_data]

        # Update plots
        min_len = min(len(self.timestamps),
                    *[len(d) for d in self.velocity_data],
                    *[len(d) for d in self.position_data],
                    *[len(d) for d in self.accel_data])
        timestamps = self.timestamps[-min_len:]

        for i in range(4):
            self.quat_curves[i].setData(timestamps, self.quat_data[i][-min_len:])
        for i in range(3):
            self.accel_curves[i].setData(timestamps, self.accel_data[i][-min_len:])
            self.velocity_curves[i].setData(timestamps, self.velocity_data[i][-min_len:])
            self.position_curves[i].setData(timestamps, self.position_data[i][-min_len:])

    def closeEvent(self, event):
        self.sample_thread.stop()
        self.device.close()
        event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    viewer = IMUMeshViewer()
    viewer.show()
    sys.exit(app.exec_())
