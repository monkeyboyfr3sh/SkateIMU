import sys
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from imu_device import UARTIMUDevice

class IMUPlotter(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Live IMU Data Plot")
        self.resize(1000, 600)

        # Setup UART IMU device
        self.device = UARTIMUDevice(port='COM7', baudrate=115200)

        # Layout
        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)

        # Quaternion plot
        self.quat_plot = pg.PlotWidget(title="Quaternion (w, x, y, z)")
        self.quat_plot.addLegend()
        self.quat_plot.setYRange(-1.2, 1.2)
        self.quat_curves = [self.quat_plot.plot(pen=pg.mkPen(c, width=2), name=label)
                            for c, label in zip(['w', 'r', 'g', 'b'], ['w', 'x', 'y', 'z'])]
        layout.addWidget(self.quat_plot)

        # Acceleration plot
        self.accel_plot = pg.PlotWidget(title="Linear Acceleration (x, y, z)")
        self.accel_plot.addLegend()
        self.accel_plot.setYRange(-20, 20)
        self.accel_curves = [self.accel_plot.plot(pen=pg.mkPen(c, width=2), name=label)
                             for c, label in zip(['r', 'g', 'b'], ['x', 'y', 'z'])]
        layout.addWidget(self.accel_plot)

        # Buffers
        self.max_points = 300
        self.timestamps = []
        self.quat_data = [[] for _ in range(4)]
        self.accel_data = [[] for _ in range(3)]

        # Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(20)  # ~30 FPS

    def update_plot(self):
        result = self.device.sample()
        if result is None:
            return

        timestamp, accel, quat = result
        self.timestamps.append(timestamp)

        # Quaternion: [w, x, y, z]
        for i in range(4):
            self.quat_data[i].append(quat[i])
        for i in range(3):
            self.accel_data[i].append(accel[i])

        # Trim to max_points
        if len(self.timestamps) > self.max_points:
            self.timestamps = self.timestamps[-self.max_points:]
            for i in range(4):
                self.quat_data[i] = self.quat_data[i][-self.max_points:]
            for i in range(3):
                self.accel_data[i] = self.accel_data[i][-self.max_points:]

        # Update plots
        for i in range(4):
            self.quat_curves[i].setData(self.timestamps, self.quat_data[i])
        for i in range(3):
            self.accel_curves[i].setData(self.timestamps, self.accel_data[i])


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = IMUPlotter()
    window.show()
    sys.exit(app.exec_())
