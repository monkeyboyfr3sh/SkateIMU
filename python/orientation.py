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
mesh.apply_scale(0.01)
mesh.vertices -= mesh.centroid
vertices = mesh.vertices
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

def update():
    quat = device.sample()

    if quat is None:
        return

    # Convert to rotation matrix (order: [x, y, z, w])
    rot = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix()

    # Apply rotation
    rotated = (rot @ vertices.T).T
    mesh_data.setVertexes(rotated)
    mesh_item.meshDataChanged()

# Timer to refresh every 50ms
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)

# Run the app
sys.exit(app.exec_())
