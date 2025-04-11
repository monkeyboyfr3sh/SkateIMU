import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5 import QtWidgets, QtCore
import sys
from imu_device import UARTIMUDevice

# Load and prepare STL
mesh = trimesh.load('C:/Users/monke/Documents/GitHub/hello_world/python/assets/Skateboard - 172002/files/Skateboard_WHOLE.stl')

print("Original size (x, y, z):", mesh.extents)

current_length = mesh.extents[0]  # Assuming X is length
desired_length = 0.81
scale_factor = desired_length / current_length
print(f"Scale factor = {scale_factor}")

mesh.apply_scale(scale_factor)
print("After scaling (x, y, z):", mesh.extents)

# Center the mesh for clean rotation
mesh.vertices -= mesh.centroid
vertices = mesh.vertices.copy()
faces = mesh.faces

# Build mesh data for pyqtgraph
mesh_data = gl.MeshData(vertexes=vertices, faces=faces)

# Create Qt App
app = QtWidgets.QApplication(sys.argv)
view = gl.GLViewWidget()
view.setWindowTitle('Real-time IMU STL Viewer')
view.setGeometry(100, 100, 800, 600)
view.show()

# Add grid
grid = gl.GLGridItem()
grid.setSize(x=2, y=2)
grid.setSpacing(0.1, 0.1)
view.addItem(grid)

# Add mesh to scene
mesh_item = gl.GLMeshItem(meshdata=mesh_data, smooth=False, drawFaces=True, drawEdges=True, edgeColor=(0, 0, 0, 1), color=(0.6, 0.8, 1, 1))
mesh_item.setGLOptions('opaque')
view.addItem(mesh_item)

# Setup IMU device
device = UARTIMUDevice(port='COM7', baudrate=115200)

# Integration state
prev_time = None
velocity = np.zeros(3)
position = np.zeros(3)

def update():
    global prev_time, velocity, position

    result = device.sample()
    if result is None:
        return

    timestamp, accel_world, quat = result

    if prev_time is None:
        prev_time = timestamp
        return

    dt = timestamp - prev_time
    prev_time = timestamp

    # Integrate acceleration -> velocity
    velocity += accel_world * dt

    # Integrate velocity -> position
    position += velocity * dt

    # Convert quaternion to rotation matrix
    rot = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix()
    rotated_vertices = (rot @ vertices.T).T

    # Apply only rotation to mesh
    mesh_data.setVertexes(rotated_vertices)
    mesh_item.meshDataChanged()

    # Center the camera on the current position
    view.opts['center'] = pg.Vector(position[0], position[1], position[2])
    
# Timer to refresh
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)

# Run the app
sys.exit(app.exec_())
