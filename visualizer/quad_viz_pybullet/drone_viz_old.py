"""
drone_viz.py — PyBullet drone visualizer with PyQtGraph live plots
Receives pose packets from MATLAB over UDP and updates the 3D drone model.
Packet format: 7 little-endian doubles [x, y, z, qw, qx, qy, qz]
UDP port: 25000
"""

import socket
import struct
import threading
import time
import os
import sys
from collections import deque

import pybullet as p
import pybullet_data

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

UDP_PORT      = 25000
PACKET_FORMAT = "<7d"
PACKET_SIZE   = struct.calcsize(PACKET_FORMAT)  # 56 bytes
MAX_POINTS    = 300

# ---------- PyBullet setup ----------
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
assets_path = os.path.expanduser("~/research/sitl/quad_viz_pybullet/gym-pybullet-drones/gym_pybullet_drones/assets/")
p.setAdditionalSearchPath(assets_path)
p.setGravity(0, 0, 0)
p.setRealTimeSimulation(0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, -1])

drone_id = p.loadURDF(assets_path + "racer.urdf",
                      basePosition=[0, 0, 1],
                      baseOrientation=[0, 0, 0, 1],
                      useFixedBase=False,
                      flags=p.URDF_USE_INERTIA_FROM_FILE)

p.resetDebugVisualizerCamera(
    cameraDistance=3.0, cameraYaw=45,
    cameraPitch=-30, cameraTargetPosition=[0, 0, 1]
)

p.addUserDebugLine([0,0,0],[1,0,0],[1,0,0], lineWidth=2)
p.addUserDebugLine([0,0,0],[0,1,0],[0,1,0], lineWidth=2)
p.addUserDebugLine([0,0,0],[0,0,1],[0,0,1], lineWidth=2)

# ---------- Shared state ----------
latest_pose  = None
pose_lock    = threading.Lock()
t_hist = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)
x_hist = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)
y_hist = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)
z_hist = deque([0.0] * MAX_POINTS, maxlen=MAX_POINTS)
t_start = time.time()

# ---------- UDP listener ----------
def udp_listener():
    global latest_pose
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(1.0)
    print(f"[drone_viz] Listening on UDP port {UDP_PORT}")
    while True:
        try:
            data, _ = sock.recvfrom(PACKET_SIZE * 2)
            if len(data) < PACKET_SIZE:
                continue
            vals = struct.unpack(PACKET_FORMAT, data[:PACKET_SIZE])
            with pose_lock:
                latest_pose = vals
        except socket.timeout:
            pass

threading.Thread(target=udp_listener, daemon=True).start()

# ---------- PyBullet loop (separate thread) ----------
prev_pos     = None
trail_points = []

def pybullet_loop():
    global prev_pos
    try:
        while p.isConnected():
            with pose_lock:
                pose = latest_pose

            if pose is not None:
                x, y, z, qw, qx, qy, qz = pose
                pos = [x, y, z]
                orn = [qx, qy, qz, qw]

                p.resetBasePositionAndOrientation(drone_id, pos, orn)
                p.resetDebugVisualizerCamera(
                    cameraDistance=3.0, cameraYaw=45,
                    cameraPitch=-30, cameraTargetPosition=pos
                )

                if prev_pos is not None:
                    line = p.addUserDebugLine(prev_pos, pos,
                                              lineColorRGB=[1, 0.6, 0],
                                              lineWidth=1.5)
                    trail_points.append(line)
                    if len(trail_points) > 200:
                        p.removeUserDebugItem(trail_points.pop(0))
                prev_pos = pos

                t_hist.append(time.time() - t_start)
                x_hist.append(x)
                y_hist.append(y)
                z_hist.append(z)

            p.stepSimulation()
            time.sleep(0.002)  # ~500 Hz cap

    except p.error:
        pass

threading.Thread(target=pybullet_loop, daemon=True).start()

# ---------- PyQtGraph window ----------
app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

pg.setConfigOptions(antialias=True, background='k', foreground='w')

win = pg.GraphicsLayoutWidget(title="Drone Position")
win.resize(500, 600)
win.setWindowTitle("Drone Position")

plots = []
curves = []
configs = [
    ("x (m)", pg.mkPen(color=(255, 80,  80),  width=2)),
    ("y (m)", pg.mkPen(color=(80,  220, 80),  width=2)),
    ("z (m)", pg.mkPen(color=(80,  140, 255), width=2)),
]

for i, (label, pen) in enumerate(configs):
    plt = win.addPlot(row=i, col=0)
    plt.setLabel('left', label)
    plt.showGrid(x=True, y=True, alpha=0.3)
    plt.setMouseEnabled(x=False, y=False)
    if i < 2:
        plt.hideAxis('bottom')
    else:
        plt.setLabel('bottom', 'time (s)')
    curve = plt.plot(pen=pen)
    plots.append(plt)
    curves.append(curve)

win.show()

def update_plots():
    t = list(t_hist)
    for curve, hist in zip(curves, [x_hist, y_hist, z_hist]):
        curve.setData(t, list(hist))

timer = QtCore.QTimer()
timer.timeout.connect(update_plots)
timer.start(50)  # 20 Hz plot refresh

print("[drone_viz] Running. Close either window to exit.")
app.exec()
print("[drone_viz] Done.")