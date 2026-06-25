"""
drone_viz.py — PyBullet drone visualizer with PyQtGraph live plots
Receives full-state + controller-target packets from MATLAB over UDP.

Packet layout: 28 little-endian doubles (224 bytes)
  [0:2]   ENU position          x, y, z                    (m)
  [3:6]   quaternion            qw, qx, qy, qz
  [7:9]   NED velocity          vN, vE, vD                 (m/s)
  [10:12] velocity commands     vx_cmd, vy_cmd, vz_cmd     (m/s)
  [13:15] attitude              roll, pitch, yaw            (deg)
  [16:17] desired angles        phi_des, theta_des          (deg)
  [18:20] gyro rates            p, q, r                    (deg/s)
  [21:23] rate commands         p_cmd, q_cmd, r_cmd        (deg/s)
  [24:26] target ENU position   tx, ty, tz                 (m)
  [27]    target yaw                                        (deg)

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

# ── Config ────────────────────────────────────────────────────────────────────
UDP_PORT      = 25000
N_FIELDS      = 28
PACKET_FORMAT = f"<{N_FIELDS}d"
PACKET_SIZE   = struct.calcsize(PACKET_FORMAT)   # 224 bytes
MAX_POINTS    = 1200   # ~60 s at 20 Hz refresh

# ── PyBullet setup ────────────────────────────────────────────────────────────
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

assets_path = os.path.expanduser(
    "~/research/sitl/quad_viz_pybullet/gym-pybullet-drones/gym_pybullet_drones/assets/")
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
    cameraPitch=-30, cameraTargetPosition=[0, 0, 1])

p.addUserDebugLine([0, 0, 0], [1, 0, 0], [1, 0, 0], lineWidth=2)
p.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 1, 0], lineWidth=2)
p.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 0, 1], lineWidth=2)

# ── Shared state ──────────────────────────────────────────────────────────────
latest_fields = None
pose_lock     = threading.Lock()
t_start       = time.time()

def _dq(v=0.0):
    return deque([v] * MAX_POINTS, maxlen=MAX_POINTS)

t_hist = _dq()

# [0:2] ENU position
x_hist  = _dq(); y_hist  = _dq(); z_hist  = _dq()
# [24:26] target ENU position
tx_hist = _dq(); ty_hist = _dq(); tz_hist = _dq()

# [7:9] NED velocity
vn_hist = _dq(); ve_hist = _dq(); vd_hist = _dq()
# [10:12] velocity commands
vn_cmd_hist = _dq(); ve_cmd_hist = _dq(); vd_cmd_hist = _dq()

# [13:15] attitude (deg)
roll_hist  = _dq(); pitch_hist = _dq(); yaw_hist  = _dq()
# [16:17] desired angles (deg) — target for attitude loop
phi_des_hist   = _dq(); theta_des_hist = _dq()
# [27] target yaw (deg)
tyaw_hist = _dq()

# [18:20] gyro rates (deg/s)
p_hist = _dq(); q_hist = _dq(); r_hist = _dq()
# [21:23] rate commands (deg/s)
p_cmd_hist = _dq(); q_cmd_hist = _dq(); r_cmd_hist = _dq()

# ── UDP listener ──────────────────────────────────────────────────────────────
def udp_listener():
    global latest_fields
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(1.0)
    print(f"[drone_viz] Listening on UDP:{UDP_PORT}  (packet={PACKET_SIZE} B, {N_FIELDS} doubles)")
    while True:
        try:
            data, _ = sock.recvfrom(PACKET_SIZE * 2)
            if len(data) < PACKET_SIZE:
                continue
            vals = struct.unpack(PACKET_FORMAT, data[:PACKET_SIZE])
            with pose_lock:
                latest_fields = vals
        except socket.timeout:
            pass

threading.Thread(target=udp_listener, daemon=True).start()

# ── PyBullet loop ─────────────────────────────────────────────────────────────
prev_pos     = None
trail_points = []

def pybullet_loop():
    global prev_pos
    try:
        while p.isConnected():
            with pose_lock:
                fields = latest_fields

            if fields is not None:
                # unpack by index
                x, y, z                         = fields[0],  fields[1],  fields[2]
                qw, qx, qy, qz                  = fields[3],  fields[4],  fields[5],  fields[6]
                vn, ve, vd                       = fields[7],  fields[8],  fields[9]
                vn_cmd, ve_cmd, vd_cmd           = fields[10], fields[11], fields[12]
                roll, pitch, yaw                 = fields[13], fields[14], fields[15]
                phi_des, theta_des               = fields[16], fields[17]
                gp, gq, gr                       = fields[18], fields[19], fields[20]
                p_cmd, q_cmd, r_cmd              = fields[21], fields[22], fields[23]
                tx, ty, tz                       = fields[24], fields[25], fields[26]
                tyaw                             = fields[27]

                pos = [x, y, z]
                orn = [qx, qy, qz, qw]
                p.resetBasePositionAndOrientation(drone_id, pos, orn)
                p.resetDebugVisualizerCamera(
                    cameraDistance=4.0, cameraYaw=45,
                    cameraPitch=-30, cameraTargetPosition=pos)

                if prev_pos is not None:
                    line = p.addUserDebugLine(prev_pos, pos,
                                              lineColorRGB=[1, 0.6, 0],
                                              lineWidth=1.5)
                    trail_points.append(line)
                    if len(trail_points) > 300:
                        p.removeUserDebugItem(trail_points.pop(0))
                prev_pos = pos

                now = time.time() - t_start
                t_hist.append(now)

                x_hist.append(x);   y_hist.append(y);   z_hist.append(z)
                tx_hist.append(tx); ty_hist.append(ty); tz_hist.append(tz)

                vn_hist.append(vn); ve_hist.append(ve); vd_hist.append(vd)
                vn_cmd_hist.append(vn_cmd); ve_cmd_hist.append(ve_cmd); vd_cmd_hist.append(vd_cmd)

                roll_hist.append(roll);   pitch_hist.append(pitch); yaw_hist.append(yaw)
                phi_des_hist.append(phi_des); theta_des_hist.append(theta_des)
                tyaw_hist.append(tyaw)

                p_hist.append(gp);     q_hist.append(gq);     r_hist.append(gr)
                p_cmd_hist.append(p_cmd); q_cmd_hist.append(q_cmd); r_cmd_hist.append(r_cmd)

            p.stepSimulation()
            time.sleep(0.002)

    except p.error:
        pass

threading.Thread(target=pybullet_loop, daemon=True).start()

# ── PyQtGraph window ──────────────────────────────────────────────────────────
app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
pg.setConfigOptions(antialias=True, background='k', foreground='w')

win = pg.GraphicsLayoutWidget(title="Drone Full-State Monitor")
win.resize(760, 1100)
win.setWindowTitle("Drone Full-State Monitor")

# Pen helpers
R = (255, 80,  80)
G = (80,  220, 80)
B = (80,  140, 255)
Y = (255, 220, 50)

def solid(rgb,  w=2):   return pg.mkPen(color=rgb, width=w)
def dashed(rgb, w=1.5): return pg.mkPen(color=rgb, width=w,
                                         style=QtCore.Qt.PenStyle.DashLine)

def add_plot(row, y_label, show_bottom=False):
    plt = win.addPlot(row=row, col=0)
    plt.setLabel('left', y_label)
    plt.showGrid(x=True, y=True, alpha=0.25)
    plt.setMouseEnabled(x=False, y=False)
    if show_bottom:
        plt.setLabel('bottom', 'time (s)')
    else:
        plt.hideAxis('bottom')
    return plt

# ── Row 0: Position ───────────────────────────────────────────────────────────
plt_pos = add_plot(0, 'Position (m)')
c_x  = plt_pos.plot(pen=solid(R),  name='x')
c_y  = plt_pos.plot(pen=solid(G),  name='y')
c_z  = plt_pos.plot(pen=solid(B),  name='z')
c_tx = plt_pos.plot(pen=dashed(R), name='x tgt')
c_ty = plt_pos.plot(pen=dashed(G), name='y tgt')
c_tz = plt_pos.plot(pen=dashed(B), name='z tgt')
leg = plt_pos.addLegend(offset=(5, 5))
for c, lbl in [(c_x,'x'),(c_tx,'x tgt'),(c_y,'y'),(c_ty,'y tgt'),(c_z,'z'),(c_tz,'z tgt')]:
    leg.addItem(c, lbl)

# ── Row 1: Velocity ───────────────────────────────────────────────────────────
plt_vel = add_plot(1, 'Velocity (m/s)')
c_vn     = plt_vel.plot(pen=solid(R),  name='vN')
c_ve     = plt_vel.plot(pen=solid(G),  name='vE')
c_vd     = plt_vel.plot(pen=solid(B),  name='vD')
c_vn_cmd = plt_vel.plot(pen=dashed(R), name='vN cmd')
c_ve_cmd = plt_vel.plot(pen=dashed(G), name='vE cmd')
c_vd_cmd = plt_vel.plot(pen=dashed(B), name='vD cmd')
leg_vel = plt_vel.addLegend(offset=(5, 5))
for c, lbl in [(c_vn,'vN'),(c_vn_cmd,'vN cmd'),
               (c_ve,'vE'),(c_ve_cmd,'vE cmd'),
               (c_vd,'vD'),(c_vd_cmd,'vD cmd')]:
    leg_vel.addItem(c, lbl)

# ── Row 2: Attitude ───────────────────────────────────────────────────────────
# roll actual vs phi_des, pitch actual vs theta_des, yaw actual vs yaw target
plt_att = add_plot(2, 'Attitude (deg)')
c_roll      = plt_att.plot(pen=solid(R),  name='roll')
c_phi_des   = plt_att.plot(pen=dashed(R), name='roll tgt')
c_pitch     = plt_att.plot(pen=solid(G),  name='pitch')
c_theta_des = plt_att.plot(pen=dashed(G), name='pitch tgt')
c_yaw       = plt_att.plot(pen=solid(B),  name='yaw')
c_tyaw      = plt_att.plot(pen=dashed(B), name='yaw tgt')
leg_att = plt_att.addLegend(offset=(5, 5))
for c, lbl in [(c_roll,'roll'),(c_phi_des,'roll tgt'),
               (c_pitch,'pitch'),(c_theta_des,'phi tgt'),
               (c_yaw,'yaw'),(c_tyaw,'yaw tgt')]:
    leg_att.addItem(c, lbl)

# ── Row 3: Gyro rates ─────────────────────────────────────────────────────────
plt_gyro = add_plot(3, 'Gyro rates (deg/s)', show_bottom=True)
c_p     = plt_gyro.plot(pen=solid(R),  name='p')
c_q     = plt_gyro.plot(pen=solid(G),  name='q')
c_r     = plt_gyro.plot(pen=solid(B),  name='r')
c_p_cmd = plt_gyro.plot(pen=dashed(R), name='p cmd')
c_q_cmd = plt_gyro.plot(pen=dashed(G), name='q cmd')
c_r_cmd = plt_gyro.plot(pen=dashed(B), name='r cmd')
leg_gyro = plt_gyro.addLegend(offset=(5, 5))
for c, lbl in [(c_p,'p'),(c_p_cmd,'p cmd'),
               (c_q,'q'),(c_q_cmd,'q cmd'),
               (c_r,'r'),(c_r_cmd,'r cmd')]:
    leg_gyro.addItem(c, lbl)

win.show()

# ── Plot update ───────────────────────────────────────────────────────────────
def update_plots():
    t = list(t_hist)

    c_x.setData(t, list(x_hist));   c_tx.setData(t, list(tx_hist))
    c_y.setData(t, list(y_hist));   c_ty.setData(t, list(ty_hist))
    c_z.setData(t, list(z_hist));   c_tz.setData(t, list(tz_hist))

    c_vn.setData(t, list(vn_hist));      c_vn_cmd.setData(t, list(vn_cmd_hist))
    c_ve.setData(t, list(ve_hist));      c_ve_cmd.setData(t, list(ve_cmd_hist))
    c_vd.setData(t, list(vd_hist));      c_vd_cmd.setData(t, list(vd_cmd_hist))

    c_roll.setData(t,      list(roll_hist))
    c_phi_des.setData(t,   list(phi_des_hist))
    c_pitch.setData(t,     list(pitch_hist))
    c_theta_des.setData(t, list(theta_des_hist))
    c_yaw.setData(t,       list(yaw_hist))
    c_tyaw.setData(t,      list(tyaw_hist))

    c_p.setData(t, list(p_hist));        c_p_cmd.setData(t, list(p_cmd_hist))
    c_q.setData(t, list(q_hist));        c_q_cmd.setData(t, list(q_cmd_hist))
    c_r.setData(t, list(r_hist));        c_r_cmd.setData(t, list(r_cmd_hist))

timer = QtCore.QTimer()
timer.timeout.connect(update_plots)
timer.start(50)   # 20 Hz

print("[drone_viz] Running. Close either window to exit.")
app.exec()
print("[drone_viz] Done.")