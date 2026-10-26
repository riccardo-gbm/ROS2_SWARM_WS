# ROS2_SWARM_WS

Ground station for the **SWARM – RSCL outdoor formation experiment**: four Holybro X500 V2
quadcopters flown as a swarm, with RTK-grade pose streamed into ROS 2.

RSCL owns the leader–follower formation controller, which already works in simulation and indoors
under motion capture. This workspace is the **outdoor replacement for the MOCAP room**. The target
interface is deliberately small — the formation controller should only ever see
`/droneN/odom` (`nav_msgs/Odometry`) coming in and `/droneN/cmd_vel` (`geometry_msgs/Twist`) going
out, with MAVLink, WiFi, RTK and origin math hidden inside this stack.

The full specification is in [`RCSL Collab.pdf`](RCSL%20Collab.pdf) in the repo root.

## Hardware

Four X500 V2 airframes with Pixhawk 6C running ArduCopter, each carrying an F9P RTK receiver and an
ESP8266 WiFi bridge. Ground station is Ubuntu 22.04 running ROS 2 Humble.

| Namespace | Name    | sysid | ESP8266 IP     | Ground-station port |
| --------- | ------- | ----- | -------------- | ------------------- |
| `drone1`  | PAPOI   | 19    | 192.168.50.11  | 14551               |
| `drone2`  | TRINEO  | 17    | 192.168.50.12  | 14552               |
| `drone3`  | BISUU   | 32    | 192.168.50.13  | 14553               |
| `drone4`  | LUCAS   | 31    | 192.168.50.14  | 14554               |

All four join the router `vanttec_drones_swarm` on static IPs assigned by MAC address, kept below
`192.168.50.100` so they stay out of the DHCP range. The ESP8266 is wired to TELEM2 at 500000 baud
and listens on port 14555.

## Packages and files

### `src/swarm_bringup` — launch and configuration

- **`launch/mavros_x4.launch.py`** — brings up the whole link layer. Includes MAVROS's `node.launch`
  four times, one instance per drone, each with its own namespace, an `fcu_url` of
  `udp://:<port>@<esp_ip>:14555`, and `tgt_system` set to that drone's sysid so the four links don't
  mix. Each instance gets the shared plugin allowlist plus MAVROS's stock `apm_config.yaml`. It then
  starts `stream_setup` with the four namespaces at 25 Hz.
  The `DRONES` list at the top of the file is the single place to edit when a drone's IP or sysid
  changes.

- **`launch/mavros_sitl_x4.launch.py`** — the same four MAVROS instances pointed at ArduPilot SITL
  on localhost instead of the ESP8266 bridges, using the sysids SITL assigns with `--auto-sysid`
  (1–4) rather than the aircraft's own. Connects to SITL's TCP ports 5760–5790 by default; pass
  `use_mavproxy:=true` to use UDP 14551–14554 instead, when SITL is running with MAVProxy
  forwarding. See [Simulation](#simulation-ardupilot-sitl--gazebo).

- **`config/swarm_pluginlists.yaml`** — MAVROS plugin allowlist. It denies `*` and then allows only
  the eleven plugins the platform actually uses: `sys_status`, `sys_time`, `command`, `imu`,
  `local_position`, `global_position`, `setpoint_velocity`, `setpoint_raw`, `param`, `gps_rtk` and
  `gps_status`. This cuts each drone from several hundred topics down to roughly fifty, which
  matters because every one of them crosses a 2.4 GHz ESP8266 telemetry link. The `/**:` wildcard
  applies it to every MAVROS node.

- **`setup.py`** — installs `launch/` and `config/` into `share/` so `FindPackageShare` can resolve
  them at launch time.

- **`package.xml`** — depends on `launch`, `launch_ros` and `mavros`; exec-depends on `swarm_nodes`.

### `src/swarm_nodes` — runtime nodes

- **`swarm_nodes/stream_setup.py`** → `ros2 run swarm_nodes stream_setup`
  Requests MAVLink message rates from every drone through its
  `/droneN/mavros/set_message_interval` service: message ids 30 `ATTITUDE`,
  31 `ATTITUDE_QUATERNION`, 32 `LOCAL_POSITION_NED` and 33 `GLOBAL_POSITION_INT`, all at 25 Hz.
  It watches each `/droneN/mavros/state`, requests the rates when a drone connects, re-requests them
  on reconnect, and reapplies them every 30 s — so a flight-controller reboot or a ground control
  station overriding stream rates cannot silently drop the pose rate.
  Parameters: `namespaces`, `message_ids`, `rate_hz`, `reapply_period`.
  Started automatically by the launch file.

- **`swarm_nodes/link_monitor.py`** → `ros2 run swarm_nodes link_monitor`
  Subscribes to `/droneN/mavros/imu/data` on all four drones, counts messages and tracks the largest
  gap between consecutive ones, then logs a single line per drone every 5 s and resets the counters:

  ```
  PAPOI:  24.8 Hz, max gap  112 ms | TRINEO:  25.0 Hz, max gap   48 ms | ...
  ```

  Use it to judge WiFi link health before and during a flight.

- **`swarm_nodes/orientation_monitor.py`** → `ros2 run swarm_nodes orientation_monitor`
  Subscribes to `/droneN/mavros/imu/data` and `/droneN/mavros/global_position/compass_hdg`. It
  converts the IMU quaternion to roll/pitch/yaw, turns the ENU yaw into a compass heading with
  `(90 - yaw) % 360`, and prints roll, pitch, heading from the IMU and heading from the compass for
  all four drones once a second.
  Use it to confirm the four agree and to catch a badly calibrated compass: RTK fixes position but
  not heading, so compass error turns directly into formation yaw error.

- **`swarm_nodes/odom_relay.py`** → `ros2 run swarm_nodes odom_relay`
  Subscribes `/droneN/mavros/local_position/odom` on all four drones and republishes each one on
  **`/droneN/odom`** — the topic the formation controller subscribes to — at 25 Hz, rewriting
  `child_frame_id` to `droneN/base_link`. It keeps only the newest pose per drone and clears it
  after publishing, so a stale pose is never re-sent.
  Parameters: `namespaces`, `rate_hz`.

  **This is a passthrough.** Each pose is still expressed in that drone's own EKF origin, set
  wherever it first got a good GPS fix, so the four frames are not comparable yet. The shared
  `swarm_origin` shift is not implemented — until it is, `frame_id` is left exactly as MAVROS set it
  rather than claiming to be `swarm_origin`. The node logs this warning on startup.

- **`swarm_nodes/teleop_velocity.py`** → `ros2 run swarm_nodes teleop_velocity`
  Keyboard velocity teleop for a single drone, for bench and SITL testing. Publishes a
  `TwistStamped` to `<drone_id>/mavros/setpoint_velocity/cmd_vel` while a key is held:
  `w`/`s` for ±x, `a`/`d` for ∓y, `q`/`e` for ∓z, left/right arrows for yaw rate.
  Parameters: `drone_id` (**required**, e.g. `drone1`), `speed` (0.5 m/s), `yaw_rate` (0.5 rad/s),
  `rate_hz` (20), `hold_timeout` (0.5 s).

  ```bash
  ros2 run swarm_nodes teleop_velocity --ros-args -p drone_id:=drone1
  ```

  Two things to know before using it. **The axes are world ENU, not body** — MAVROS interprets the
  twist in the world frame regardless of `header.frame_id`, so `w` means *east*, not *forward*, no
  matter where the drone is pointing. And it publishes **straight to MAVROS with no speed clamp and
  no watchdog**; it is a test tool, not the velocity interface the formation controller will use.
  Run it in SITL first, and on a real aircraft only with an RC safety pilot and the geofence on.

- **`setup.py` / `package.xml`** — console-script entry points and dependencies (`rclpy`,
  `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `mavros_msgs`).

All three nodes subscribe with `qos_profile_sensor_data` (best-effort), matching the QoS that MAVROS
publishes its sensor topics with.

## Simulation (ArduPilot SITL + Gazebo)

The four aircraft can be replaced by ArduPilot SITL, which is useful because
`--auto-offset-line` spawns each copter with **its own EKF origin** — the same frame mismatch you
get outdoors, reproducible on a laptop without flying.

Gazebo runs in the `humble` container (that's where `gz` and the headers are); ArduPilot SITL runs
on the host (its binary is built against the host's glibc). They reach each other over localhost
because the container shares the host network namespace.

| Instance | sysid | FDM (SITL↔gz) | MAVLink TCP |
| -------- | ----- | ------------- | ----------- |
| `-I0`    | 1     | 9002 / 9003   | 5760        |
| `-I1`    | 2     | 9012 / 9013   | 5770        |
| `-I2`    | 3     | 9022 / 9023   | 5780        |
| `-I3`    | 4     | 9032 / 9033   | 5790        |

One-time setup of the ArduPilot↔Gazebo bridge plugin, **inside the container**:

```bash
mkdir -p ~/gz_ws/src && cd ~/gz_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo && export GZ_VERSION=harmonic
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build -j$(nproc)
```

`GZ_SIM_SYSTEM_PLUGIN_PATH` must contain `.../ardupilot_gazebo/build`, and `GZ_SIM_RESOURCE_PATH`
its `models` and `worlds` directories. Verify one vehicle works before trying four.

Then, on the host:

```bash
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --count 4 --auto-sysid --auto-offset-line 90,10 -L CMAC --no-mavproxy
```

And in the container, in place of `mavros_x4.launch.py`:

```bash
ros2 launch swarm_bringup mavros_sitl_x4.launch.py
```

Everything downstream — `odom_relay`, `link_monitor`, `orientation_monitor`, `teleop_velocity` —
works unchanged against the simulator, because the namespaces and topic names are identical.

## Build and run

Prerequisites: Ubuntu 22.04 with ROS 2 Humble, plus
`sudo apt install ros-humble-mavros ros-humble-mavros-extras` and the MAVROS GeographicLib datasets
script.

```bash
cd ~/Documents/VANTTEC/ROS2_SWARM_WS
colcon build --symlink-install
source install/setup.bash

# 4x MAVROS + stream_setup
ros2 launch swarm_bringup mavros_x4.launch.py

# in other terminals
ros2 run swarm_nodes link_monitor
ros2 run swarm_nodes orientation_monitor
```
