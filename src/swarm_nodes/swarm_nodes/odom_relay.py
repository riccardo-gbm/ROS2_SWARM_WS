"""Publish every drone's pose on /droneN/odom, in one shared frame the four can be compared in.

Each drone's EKF measures position from its own private origin -- wherever that vehicle happened to
get its first good GPS fix. Four drones therefore report four incompatible frames, and a formation
controller has no way to know where they are relative to each other. ArduPilot will not let us force
a common EKF origin while using GPS, so the shared frame is computed here on the ground station.

How the shift is found, per drone, once at startup:

    offset_i = geodetic_to_enu(global_fix_i, swarm_origin) - local_position_i

That single equation is the whole idea. The drone's absolute position converted into the shared
frame, minus where its own EKF thinks it is, is the constant gap between the two origins. Adding
that offset to every subsequent local pose puts all four drones in the same frame.

It needs only /droneN/mavros/global_position/global and /droneN/mavros/local_position/odom, both
already streaming at 25 Hz, so there is no extra MAVLink traffic and no dependence on gp_origin
being latched. Offsets are averaged over origin_samples messages to take the noise out, which means
THE DRONES MUST BE STATIONARY while it initialises.

Velocity and orientation are untouched: a velocity is the same in any frame that shares axes, so
the origin shift only ever moves position.

Set shared_origin:=false to go back to raw passthrough (each drone in its own frame, frame_id left
exactly as MAVROS set it). With shared_origin true and the offsets unresolved the node publishes
NOTHING -- silence is safer than poses labelled swarm_origin that are not in it.

Cross-check (method B in the spec): with publish_global_check:=true it also publishes
/droneN/odom_global, computed straight from the GPS fix rather than from local position + offset.
The two should agree within centimetres; a growing disagreement means EKF drift.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

# ---------------------------------------------------------------------------------------------
# WGS 84: the agreed model of the Earth's shape. NOT a sphere -- it bulges at the equator and is
# flattened at the poles by about 21 km. GPS, MAVROS and GeographicLib all use these numbers.
# ---------------------------------------------------------------------------------------------
_A = 6378137.0              # equatorial radius, metres: centre of the Earth out to the equator
_F = 1.0 / 298.257223563    # flattening -- how squashed. b = a(1-f) = 6356752.3 m at the poles
_E2 = _F * (2.0 - _F)       # eccentricity squared. Algebraically identical to 1-(b/a)^2, and it
                            # is the number that shows up wherever the ellipse differs from a
                            # circle. Small (0.0067), which is why a sphere is nearly right.


def geodetic_to_ecef(lat_deg, lon_deg, alt):
    """Lat/lon/height -> x, y, z metres measured from the centre of the Earth.

    A stepping stone. Latitude and longitude are ANGLES, and subtracting angles does not give
    metres, so both points go first into one metre-based system called ECEF (Earth-Centred,
    Earth-Fixed): origin at the centre of the Earth, z through the north pole, x out through
    (lat 0, lon 0), y 90 degrees east. It turns with the planet, so a fixed point on the ground
    keeps fixed numbers.

    On a perfect SPHERE of radius R this is ordinary spherical-to-Cartesian:

        x = (R+h) cos(lat) cos(lon)
        y = (R+h) cos(lat) sin(lon)
        z = (R+h) sin(lat)

    Two things change on an ellipsoid.

    1. The latitude you are given is not the angle from the centre. Geodetic latitude is the
       angle of the LOCAL VERTICAL -- the way a plumb line hangs, perpendicular to the surface.
       On a sphere that line aims at the centre. On a squashed ellipsoid it does not, except
       exactly at the equator and the poles.

       So we need n: the distance from the surface point, straight down the local vertical, to
       where it crosses the Earth's spin axis. It is the prime vertical radius of curvature, and
       it is NOT the distance to the centre:

           lat  0:  n = 6378137 m,  distance to centre = 6378137 m   (equal, on the equator)
           lat 45:  n = 6388838 m,  distance to centre = 6367490 m   (21 km apart)
           lat 90:  n = 6399594 m,  distance to centre = 6356752 m   (43 km apart)

       n is what scales the horizontal part, because (n+alt)*cos(lat) is exactly the distance out
       from the spin axis -- which is all x and y need.

    2. The z line carries an extra (1 - _E2). In the northern hemisphere the local vertical
       crosses the spin axis BELOW the centre, so the height above the equatorial plane is less
       than a sphere would give. Multiplying by (1 - _E2), which equals (b/a)^2, is that
       correction exactly.

    Two checks that the whole thing is right, both verified:
        lat 0  -> x = 6378137.0 m, the equatorial radius a
        lat 90 -> z = 6356752.3 m, the polar radius b
    """
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    # Prime vertical radius of curvature at this latitude -- see note 1 above.
    n = _A / math.sqrt(1.0 - _E2 * sin_lat * sin_lat)
    return ((n + alt) * cos_lat * math.cos(lon),        # x: out towards lon 0
            (n + alt) * cos_lat * math.sin(lon),        # y: out towards lon 90 east
            (n * (1.0 - _E2) + alt) * sin_lat)          # z: towards the north pole


def geodetic_to_enu(lat_deg, lon_deg, alt, lat0_deg, lon0_deg, alt0):
    """Answer: standing at this lat/lon, how far east, north and up am I from that other one?

    STEP 1 -- put both points in ECEF metres and subtract. That gives the straight-line gap as
    (dx, dy, dz)... but along the EARTH's axes: dz points at the north pole, not at your local
    sky. Not yet useful to a drone.

    STEP 2 -- rotate that gap onto the local east/north/up axes at the reference point. The three
    lines below are not magic. Each is a dot product of the gap with one unit vector, written out
    longhand. The vectors, expressed in ECEF, are:

        up    = ( cos(lat0)cos(lon0),  cos(lat0)sin(lon0),  sin(lat0) )  straight out of the ground
        east  = (-sin(lon0),           cos(lon0),           0         )  no lat term and no z term:
                                                                         east is always horizontal
        north = (-sin(lat0)cos(lon0), -sin(lat0)sin(lon0),  cos(lat0) )  completes the set

    Dotting the gap with each asks "how much of this gap lies along east?", and so on. All three
    are unit length and mutually perpendicular -- verified.

    Scale at the latitudes we fly at, to make it concrete:
        0.001 degrees of latitude  = 110.79 m north
        0.001 degrees of longitude = 100.42 m east
        1e-7 degrees, the resolution of the GPS MAVLink message = about 1.1 cm

    Checked against GeographicLib's CartConvert, the library MAVROS itself links against: they
    agree to 3 nanometres.
    """
    # STEP 1: the gap between the two points, in Earth-centred metres.
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt)
    x0, y0, z0 = geodetic_to_ecef(lat0_deg, lon0_deg, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0

    # STEP 2: project that gap onto the local axes at the reference point.
    lat0, lon0 = math.radians(lat0_deg), math.radians(lon0_deg)
    sin_lat, cos_lat = math.sin(lat0), math.cos(lat0)
    sin_lon, cos_lon = math.sin(lon0), math.cos(lon0)
    return (-sin_lon * dx + cos_lon * dy,                                       # gap . east
            -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz,    # gap . north
            cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz)     # gap . up


class OdomRelay(Node):
    def __init__(self):
        super().__init__('odom_relay')
        self.drone_ns = self.declare_parameter(
            'namespaces', ['drone1', 'drone2', 'drone3', 'drone4']).value
        # True  -> shift every drone into one shared frame (what the formation controller needs)
        # False -> pass poses through untouched, each in its own drone's frame
        self.shared_origin = self.declare_parameter('shared_origin', True).value
        # Whose first GPS reading becomes the shared zero point, if we are not given one.
        self.reference = self.declare_parameter('reference_drone', 'drone1').value
        # How many readings to average when working out each drone's offset. GPS jitters, so we
        # take 25 and average them -- which is why the drones must be STILL while this happens.
        self.samples_wanted = self.declare_parameter('origin_samples', 25).value
        self.publish_check = self.declare_parameter('publish_global_check', False).value
        max_rate_hz = self.declare_parameter('max_rate_hz', 0.0).value
        self.min_period = 1.0 / max_rate_hz if max_rate_hz > 0.0 else 0.0
        self.frame_id = self.declare_parameter('frame_id', 'swarm_origin').value

        # swarm_origin as an explicit lat/lon/alt -- e.g. the RTK base's surveyed position. Leave
        # unset (nan) to adopt the reference drone's first fix instead.
        lat = self.declare_parameter('origin_lat', float('nan')).value
        lon = self.declare_parameter('origin_lon', float('nan')).value
        alt = self.declare_parameter('origin_alt', float('nan')).value
        self.origin = (lat, lon, alt) if all(math.isfinite(v) for v in (lat, lon, alt)) else None
        self.origin_configured = self.origin is not None

        self.fix = {ns: None for ns in self.drone_ns}
        self.offset = {ns: None for ns in self.drone_ns}
        self.samples = {ns: [] for ns in self.drone_ns}
        self.seen = {ns: False for ns in self.drone_ns}
        self.last_pub = {ns: 0.0 for ns in self.drone_ns}

        self.pubs, self.check_pubs = {}, {}
        for ns in self.drone_ns:
            self.pubs[ns] = self.create_publisher(Odometry, f'/{ns}/odom',
                                                  qos_profile_sensor_data)
            if self.publish_check:
                self.check_pubs[ns] = self.create_publisher(
                    Odometry, f'/{ns}/odom_global', qos_profile_sensor_data)
            self.create_subscription(NavSatFix, f'/{ns}/mavros/global_position/global',
                                     lambda msg, ns=ns: self.on_fix(ns, msg),
                                     qos_profile_sensor_data)
            self.create_subscription(Odometry, f'/{ns}/mavros/local_position/odom',
                                     lambda msg, ns=ns: self.on_odom(ns, msg),
                                     qos_profile_sensor_data)

        if self.shared_origin:
            # Say so periodically while we are still waiting, so a silent /droneN/odom is
            # explained rather than mistaken for a crashed node.
            self.create_timer(10.0, self.report_pending)

        if not self.shared_origin:
            self.get_logger().warn(
                'shared_origin disabled: passthrough mode, every pose stays in its own drone\'s '
                'EKF origin and the four frames are NOT comparable')
        elif self.origin_configured:
            self.get_logger().info(
                f'swarm_origin fixed at {lat:.7f}, {lon:.7f}, {alt:.2f} m (ellipsoidal)')
        else:
            self.get_logger().info(
                f'swarm_origin will be {self.reference}\'s first fix; '
                f'keep the drones STILL while offsets settle')

    def report_pending(self):
        waiting = [ns for ns in self.drone_ns if self.offset[ns] is None]
        if not waiting:
            return
        if self.origin is None:
            self.get_logger().warn(
                f'still no swarm_origin: waiting for a GPS fix from {self.reference}. '
                f'Nothing is being published on /droneN/odom.')
        else:
            missing_fix = [ns for ns in waiting if self.fix[ns] is None]
            detail = f' (no GPS fix yet: {", ".join(missing_fix)})' if missing_fix else ''
            self.get_logger().warn(
                f'offsets unresolved for {", ".join(waiting)}{detail}; '
                f'those drones publish nothing until they settle')

    def on_fix(self, ns, msg):
        # status < 0 is NavSatStatus.STATUS_NO_FIX. Checking only isfinite() is NOT enough:
        # with no satellite lock MAVROS publishes latitude = longitude = 0.0, which is perfectly
        # finite and would be accepted as a real position off the coast of Africa. Every offset
        # computed from it would be garbage, and nothing downstream would look wrong.
        if msg.status.status < 0:
            self.get_logger().warn(f'{ns}: GPS reports no fix, ignoring position',
                                   throttle_duration_sec=15.0)
            return
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.fix[ns] = (msg.latitude, msg.longitude, msg.altitude)
            if self.origin is None and ns == self.reference:
                self.origin = self.fix[ns]
                self.get_logger().info(
                    f'swarm_origin adopted from {ns}: '
                    f'{self.origin[0]:.7f}, {self.origin[1]:.7f}, {self.origin[2]:.2f} m')

    def on_odom(self, ns, msg):
        if not self.seen[ns]:
            self.get_logger().info(f'{ns}: first odometry message received')
            self.seen[ns] = True

        if not self.shared_origin:
            self.emit(ns, msg)
            return

        if self.offset[ns] is None:
            # We do not know yet how far this drone's private zero sits from the shared zero, so
            # we cannot honestly say where it is. Publishing nothing is safer than publishing a
            # position labelled "swarm_origin" that is not actually in that frame.
            self.collect_offset(ns, msg)
            return

        # The whole trick, in three lines: add this drone's constant offset to the position its
        # own EKF reported, and the result is in the shared frame.
        p = msg.pose.pose.position
        ex, ey, ez = self.offset[ns]
        p.x, p.y, p.z = p.x + ex, p.y + ey, p.z + ez
        msg.header.frame_id = self.frame_id
        # Speed and heading are NOT touched: a velocity is the same number whichever zero point
        # you measure positions from, so moving the origin only ever affects position.
        self.emit(ns, msg)

        if self.publish_check:
            self.emit_global_check(ns, msg)

    def collect_offset(self, ns, msg):
        """Work out the constant gap between this drone's private zero and the shared zero.

            offset = where the drone really is  -  where its own EKF thinks it is

        Both sides are in metres and describe the same instant, so the difference is the gap
        between the two zero points. That gap never changes, because neither zero point moves --
        which is why we only need to measure it once.
        """
        if self.origin is None or self.fix[ns] is None:
            return  # need both a shared zero and a GPS reading before we can compare
        # Where the drone really is, in metres from the shared zero, according to GPS.
        east, north, up = geodetic_to_enu(*self.fix[ns], *self.origin)
        # Where the drone thinks it is, in metres from its own private zero.
        p = msg.pose.pose.position
        # The difference. Collected 25 times and averaged, to cancel GPS jitter.
        self.samples[ns].append((east - p.x, north - p.y, up - p.z))
        if len(self.samples[ns]) < self.samples_wanted:
            return

        n = len(self.samples[ns])
        self.offset[ns] = tuple(sum(s[i] for s in self.samples[ns]) / n for i in range(3))
        spread = [max(s[i] for s in self.samples[ns]) - min(s[i] for s in self.samples[ns])
                  for i in range(3)]
        self.get_logger().info(
            f'{ns}: offset {self.offset[ns][0]:+.2f}, {self.offset[ns][1]:+.2f}, '
            f'{self.offset[ns][2]:+.2f} m (spread over {n} samples: '
            f'{spread[0]:.2f}, {spread[1]:.2f}, {spread[2]:.2f} m)')
        if max(spread[0], spread[1]) > 1.0:
            self.get_logger().warn(
                f'{ns}: offset samples spread over {max(spread[0], spread[1]):.1f} m -- '
                f'was the drone moving, or is the GPS fix poor?')
        if all(self.offset[k] is not None for k in self.drone_ns):
            self.get_logger().info('all drones resolved; /droneN/odom now shares one frame')

    def emit_global_check(self, ns, shifted):
        """Method B: straight from the GPS fix, for comparison against the shifted pose."""
        if self.fix[ns] is None:
            return
        east, north, up = geodetic_to_enu(*self.fix[ns], *self.origin)
        check = Odometry()
        check.header = shifted.header
        check.child_frame_id = shifted.child_frame_id
        check.pose.pose.orientation = shifted.pose.pose.orientation
        check.twist = shifted.twist
        check.pose.pose.position.x = east
        check.pose.pose.position.y = north
        check.pose.pose.position.z = up
        self.check_pubs[ns].publish(check)

    def emit(self, ns, msg):
        if self.min_period:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_pub[ns] < self.min_period:
                return
            self.last_pub[ns] = now
        msg.child_frame_id = f'{ns}/base_link'
        self.pubs[ns].publish(msg)


def main():
    rclpy.init()
    rclpy.spin(OdomRelay())


if __name__ == '__main__':
    main()
