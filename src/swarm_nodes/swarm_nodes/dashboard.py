"""Ground-station dashboard: all four drones on one screen, with controls.

    ros2 run swarm_nodes dashboard

Running a four-drone field test from six terminals is how you miss a drone losing its link. This
puts link rate, mode, arming, GPS, battery, position and command age for every drone side by side,
colour-coded so a problem is visible without reading numbers.

Position comes from /droneN/odom, so once odom_relay has resolved the shared origin these are
directly comparable -- that is the view that tells you whether a formation is actually holding.

Controls, in increasing order of how much they can hurt:

    GUIDED / LAND    no confirmation, both are recoveries
    TAKEOFF / ARM    confirmation dialog, they spin propellers
    LAND ALL         no confirmation, deliberately -- in an emergency you do not want a dialog

Arming here has no countdown, unlike first_flight. Use first_flight for a planned launch; these
buttons are for recovering a single drone mid-session.

ROS spins in a background thread and the Qt timer reads the latest snapshot, so a stalled link
never freezes the window.
"""
import math
import signal
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from mavros_msgs.msg import GPSRAW, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (QApplication, QGridLayout, QHBoxLayout, QLabel,
                             QMessageBox, QPushButton, QVBoxLayout, QWidget)

NAMES = {'drone1': 'PAPOI', 'drone2': 'TRINEO', 'drone3': 'BISUU', 'drone4': 'LUCAS'}

OK, WARN, BAD, DIM = '#2e7d32', '#ef6c00', '#c62828', '#757575'

# Treat a drone as off the air if nothing has arrived for this long.
STALE_AFTER = 3.0
ROWS = ['link', 'mode', 'armed', 'gps', 'gps acc', 'battery', 'east', 'north', 'up', 'cmd age']

# GPS_FIX_TYPE. Anything below RTK Fixed means the formation positions are not centimetre-accurate,
# so 3D is shown as a warning rather than success -- this project's whole premise is RTK.
FIX_TYPES = {0: ('no GPS', BAD), 1: ('no fix', BAD), 2: ('2D', BAD), 3: ('3D', WARN),
             4: ('DGPS', WARN), 5: ('RTK Float', WARN), 6: ('RTK Fixed', OK),
             7: ('static', WARN), 8: ('PPP', WARN)}


class SwarmMonitor(Node):
    """Subscribes to everything the dashboard shows and owns the service clients."""

    def __init__(self, namespaces):
        super().__init__('dashboard')
        self.drone_ns = namespaces
        # 4S LiPo defaults: 14.0 V is ~3.5 V/cell, 13.2 V is ~3.3 V/cell. Override for 6S.
        self.warn_v = self.declare_parameter('battery_warn_v', 14.0).value
        self.low_v = self.declare_parameter('battery_low_v', 13.2).value
        self.lock = threading.Lock()
        self.data = {ns: {'state': None, 'odom': None, 'battery': None, 'gps': None,
                          'odom_count': 0, 'odom_window': time.monotonic(),
                          'odom_hz': None, 'last_cmd': None,
                          'last_odom': None, 'last_state': None} for ns in namespaces}

        self.mode_cli, self.arm_cli, self.takeoff_cli, self.land_cli = {}, {}, {}, {}
        for ns in namespaces:
            self.create_subscription(State, f'/{ns}/mavros/state',
                                     lambda m, ns=ns: self.on_state(ns, m),
                                     qos_profile_sensor_data)
            self.create_subscription(BatteryState, f'/{ns}/mavros/battery',
                                     lambda m, ns=ns: self.put(ns, 'battery', m),
                                     qos_profile_sensor_data)
            self.create_subscription(GPSRAW, f'/{ns}/mavros/gpsstatus/gps1/raw',
                                     lambda m, ns=ns: self.put(ns, 'gps', m),
                                     qos_profile_sensor_data)
            self.create_subscription(Odometry, f'/{ns}/odom',
                                     lambda m, ns=ns: self.on_odom(ns, m),
                                     qos_profile_sensor_data)
            self.create_subscription(Twist, f'/{ns}/cmd_vel',
                                     lambda m, ns=ns: self.put(ns, 'last_cmd',
                                                               time.monotonic()), 10)
            self.mode_cli[ns] = self.create_client(SetMode, f'/{ns}/mavros/set_mode')
            self.arm_cli[ns] = self.create_client(CommandBool, f'/{ns}/mavros/cmd/arming')
            self.takeoff_cli[ns] = self.create_client(CommandTOL, f'/{ns}/mavros/cmd/takeoff')
            self.land_cli[ns] = self.create_client(CommandTOL, f'/{ns}/mavros/cmd/land')

    def on_state(self, ns, msg):
        with self.lock:
            self.data[ns]['state'] = msg
            self.data[ns]['last_state'] = time.monotonic()

    def put(self, ns, key, value):
        # ROS messages arrive on a background thread while Qt draws on the main one, so every
        # read and write of self.data is wrapped in a lock to stop them tripping over each other.
        with self.lock:
            self.data[ns][key] = value

    def on_odom(self, ns, msg):
        with self.lock:
            d = self.data[ns]
            d['odom'] = msg
            d['last_odom'] = time.monotonic()
            d['odom_count'] += 1
            elapsed = time.monotonic() - d['odom_window']
            if elapsed >= 1.0:
                d['odom_hz'] = d['odom_count'] / elapsed
                d['odom_count'] = 0
                d['odom_window'] = time.monotonic()

    def snapshot(self):
        """Hand the drawing code a frozen copy, so the screen cannot change halfway through a
        redraw and show half-old, half-new numbers."""
        with self.lock:
            return {ns: dict(d) for ns, d in self.data.items()}

    # -- commands --------------------------------------------------------------

    def set_mode(self, ns, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.mode_cli[ns].call_async(req)
        self.get_logger().info(f'{ns}: requested {mode}')

    def arm(self, ns, value):
        req = CommandBool.Request()
        req.value = value
        self.arm_cli[ns].call_async(req)
        self.get_logger().warn(f'{ns}: requested {"ARM" if value else "DISARM"}')

    def takeoff(self, ns, altitude):
        req = CommandTOL.Request()
        req.altitude = float(altitude)
        self.takeoff_cli[ns].call_async(req)
        self.get_logger().warn(f'{ns}: requested takeoff to {altitude} m')

    def land(self, ns):
        self.land_cli[ns].call_async(CommandTOL.Request())
        self.get_logger().warn(f'{ns}: requested LAND')


class Dashboard(QWidget):
    def __init__(self, monitor):
        super().__init__()
        self.monitor = monitor
        self.cells = {}
        self.setWindowTitle('VANTTEC SWARM')
        mono = QFont('monospace', 11)

        grid = QGridLayout()
        grid.setHorizontalSpacing(18)
        for col, ns in enumerate(monitor.drone_ns, start=1):
            head = QLabel(NAMES.get(ns, ns))
            head.setFont(QFont('monospace', 13, QFont.Bold))
            head.setAlignment(Qt.AlignCenter)
            grid.addWidget(head, 0, col)

        for row, label in enumerate(ROWS, start=1):
            tag = QLabel(label)
            tag.setFont(mono)
            tag.setStyleSheet(f'color: {DIM};')
            grid.addWidget(tag, row, 0)
            for col, ns in enumerate(monitor.drone_ns, start=1):
                cell = QLabel('—')
                cell.setFont(mono)
                cell.setAlignment(Qt.AlignCenter)
                grid.addWidget(cell, row, col)
                self.cells[(ns, label)] = cell

        button_row = len(ROWS) + 1
        for col, ns in enumerate(monitor.drone_ns, start=1):
            box = QVBoxLayout()
            for text, handler in (('GUIDED', lambda _, n=ns: self.monitor.set_mode(n, 'GUIDED')),
                                  ('ARM', lambda _, n=ns: self.confirm_arm(n)),
                                  ('TAKEOFF', lambda _, n=ns: self.confirm_takeoff(n)),
                                  ('LAND', lambda _, n=ns: self.monitor.land(n))):
                button = QPushButton(text)
                button.clicked.connect(handler)
                box.addWidget(button)
            holder = QWidget()
            holder.setLayout(box)
            grid.addWidget(holder, button_row, col)

        land_all = QPushButton('■   L A N D   A L L   ■')
        land_all.setFont(QFont('monospace', 15, QFont.Bold))
        land_all.setMinimumHeight(64)
        land_all.setStyleSheet(
            f'background-color: {BAD}; color: white; border-radius: 6px;')
        land_all.clicked.connect(self.land_all)

        self.banner = QLabel('starting…')
        self.banner.setFont(mono)
        self.banner.setStyleSheet(f'color: {DIM};')

        outer = QVBoxLayout()
        outer.addLayout(grid)
        outer.addWidget(self.banner)
        row = QHBoxLayout()
        row.addWidget(land_all)
        outer.addLayout(row)
        self.setLayout(outer)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(200)

    # -- confirmations ---------------------------------------------------------

    def confirm_arm(self, ns):
        if self.ask(f'ARM {NAMES.get(ns, ns)}?',
                    'Propellers will spin immediately.\n\nArea clear? Safety pilot ready?'):
            self.monitor.arm(ns, True)

    def confirm_takeoff(self, ns):
        if self.ask(f'TAKE OFF {NAMES.get(ns, ns)}?',
                    'The drone will climb to 2.0 m.\n\nArea clear above it?'):
            self.monitor.takeoff(ns, 2.0)

    def ask(self, title, text):
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Warning)
        box.setWindowTitle(title)
        box.setText(text)
        box.setStandardButtons(QMessageBox.Yes | QMessageBox.Cancel)
        # Cancel is the highlighted button, so an absent-minded press of Enter cancels rather
        # than arming a drone.
        box.setDefaultButton(QMessageBox.Cancel)
        return box.exec_() == QMessageBox.Yes

    def land_all(self):
        # Deliberately no dialog: this is the button you press when something is going wrong.
        for ns in self.monitor.drone_ns:
            self.monitor.land(ns)

    # -- display ---------------------------------------------------------------

    @staticmethod
    def stale_for(stamp):
        """Seconds since a message last arrived, or a big number if none ever did."""
        return 1e9 if stamp is None else time.monotonic() - stamp

    def set(self, ns, row, text, colour=None):
        cell = self.cells[(ns, row)]
        cell.setText(text)
        cell.setStyleSheet(f'color: {colour};' if colour else '')

    def refresh(self):
        """Redraw every cell. Runs five times a second from a Qt timer."""
        snap = self.monitor.snapshot()
        airborne = 0
        for ns, d in snap.items():
            state, odom, batt = d['state'], d['odom'], d['battery']

            # A rate is only meaningful if messages are still arriving. Without this check a
            # drone that drops off the network keeps displaying its last good rate indefinitely.
            hz, silent = d['odom_hz'], self.stale_for(d['last_odom'])
            if hz is None:
                self.set(ns, 'link', '—', DIM)
            elif silent > STALE_AFTER:
                self.set(ns, 'link', f'DEAD {silent:.0f}s', BAD)
            else:
                self.set(ns, 'link', f'{hz:4.1f} Hz',
                         OK if hz >= 20 else WARN if hz >= 10 else BAD)

            if state is None:
                for row in ('mode', 'armed'):
                    self.set(ns, row, '—', DIM)
            elif self.stale_for(d['last_state']) > STALE_AFTER:
                self.set(ns, 'mode', 'NO DATA', BAD)
                self.set(ns, 'armed', '?', BAD)
            else:
                self.set(ns, 'mode', state.mode or '—',
                         OK if state.mode == 'GUIDED' else WARN)
                self.set(ns, 'armed', 'ARMED' if state.armed else 'disarmed',
                         BAD if state.armed else DIM)
                if not state.connected:
                    self.set(ns, 'mode', 'NO LINK', BAD)
                if state.armed:
                    airborne += 1

            gps = d['gps']
            if gps is None:
                self.set(ns, 'gps', '—', DIM)
                self.set(ns, 'gps acc', '—', DIM)
            else:
                label, colour = FIX_TYPES.get(gps.fix_type, (f'type {gps.fix_type}', WARN))
                sats = gps.satellites_visible
                self.set(ns, 'gps', f'{label} {sats if sats != 255 else "?"}', colour)
                # h_acc is in millimetres
                self.set(ns, 'gps acc', f'{gps.h_acc / 1000.0:5.2f} m',
                         OK if gps.h_acc <= 100 else WARN if gps.h_acc <= 1000 else BAD)

            if batt is None or not math.isfinite(batt.voltage):
                self.set(ns, 'battery', '—', DIM)
            else:
                # Voltage only. ArduPilot reports percentage as -1 unless BATT_CAPACITY is
                # configured, which showed up as a red "-100%" -- worse than useless.
                self.set(ns, 'battery', f'{batt.voltage:5.2f} V',
                         OK if batt.voltage >= self.monitor.warn_v
                         else WARN if batt.voltage >= self.monitor.low_v else BAD)

            if odom is None:
                for row in ('east', 'north', 'up'):
                    self.set(ns, row, '—', DIM)
            else:
                p = odom.pose.pose.position
                shared = odom.header.frame_id == 'swarm_origin'
                colour = None if shared else WARN
                self.set(ns, 'east', f'{p.x:7.2f}', colour)
                self.set(ns, 'north', f'{p.y:7.2f}', colour)
                self.set(ns, 'up', f'{p.z:7.2f}', colour)

            if d['last_cmd'] is None:
                self.set(ns, 'cmd age', '—', DIM)
            else:
                age = time.monotonic() - d['last_cmd']
                self.set(ns, 'cmd age', f'{age:5.2f}s',
                         OK if age < 0.5 else WARN if age < 2.0 else BAD)

        frames = {d['odom'].header.frame_id for d in snap.values() if d['odom']}
        if not frames:
            note = 'no odometry yet'
        elif frames == {'swarm_origin'}:
            note = 'shared frame resolved — positions are directly comparable'
        else:
            note = (f'NOT in a shared frame ({", ".join(sorted(frames))}) — '
                    f'positions are NOT comparable between drones')
        fixes = [d['gps'].fix_type for d in snap.values() if d['gps']]
        if fixes and all(f == 6 for f in fixes):
            rtk = f'RTK Fixed on all {len(fixes)}'
        elif fixes:
            rtk = f'RTK on {sum(1 for f in fixes if f == 6)}/{len(fixes)} — NOT all centimetre'
        else:
            rtk = 'no GPS data'
        self.banner.setText(f'{note}    |    {rtk}    |    {airborne} armed')
        good = frames == {'swarm_origin'} and bool(fixes) and all(f == 6 for f in fixes)
        self.banner.setStyleSheet(f'color: {OK if good else WARN};')


def main():
    namespaces = ['drone1', 'drone2', 'drone3', 'drone4']
    rclpy.init()
    monitor = SwarmMonitor(namespaces)

    executor = MultiThreadedExecutor()
    executor.add_node(monitor)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)   # Ctrl-C in the terminal closes the window
    window = Dashboard(monitor)
    window.resize(760, 520)
    window.show()
    try:
        app.exec_()
    finally:
        executor.shutdown()
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
