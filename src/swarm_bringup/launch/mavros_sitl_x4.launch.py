"""Bring up four MAVROS instances against ArduPilot SITL instead of the real aircraft.

Mirrors mavros_x4.launch.py, but points at localhost SITL rather than the ESP8266 bridges, and uses
the sysids SITL assigns with --auto-sysid (1-4) rather than the aircraft's own (19/17/32/31).

Start SITL first, on the host:

    ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \\
        --count 4 --auto-sysid --auto-offset-line 90,10 -L CMAC --no-mavproxy

Then here:

    ros2 launch swarm_bringup mavros_sitl_x4.launch.py

With --no-mavproxy, SITL instance i serves MAVLink on TCP 5760 + 10*i. If you want the MAVProxy
console instead, drop --no-mavproxy, give each instance --out=udp:127.0.0.1:1455N, and launch with
use_mavproxy:=true.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DRONES = [  # (namespace, sysid, sitl_tcp_port, mavproxy_udp_port)
    ('drone1', 1, 5760, 14551),
    ('drone2', 2, 5770, 14552),
    ('drone3', 3, 5780, 14553),
    ('drone4', 4, 5790, 14554),
]


def launch_setup(context, *args, **kwargs):
    use_mavproxy = LaunchConfiguration('use_mavproxy').perform(context) == 'true'

    node_launch = PathJoinSubstitution([FindPackageShare('mavros'), 'launch', 'node.launch'])
    apm_config = PathJoinSubstitution([FindPackageShare('mavros'), 'launch', 'apm_config.yaml'])
    plugins = PathJoinSubstitution(
        [FindPackageShare('swarm_bringup'), 'config', 'swarm_pluginlists.yaml'])

    mavros = [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(node_launch),
            launch_arguments={
                # MAVProxy pushes to a UDP port; without it we talk to SITL's TCP server directly.
                'fcu_url': f'udp://:{udp_port}@' if use_mavproxy else f'tcp://127.0.0.1:{tcp_port}',
                'gcs_url': '',
                'tgt_system': str(sysid),
                'tgt_component': '1',
                'pluginlists_yaml': plugins,
                'config_yaml': apm_config,
                'namespace': ns,
            }.items())
        for ns, sysid, tcp_port, udp_port in DRONES]

    streams = Node(
        package='swarm_nodes', executable='stream_setup', name='stream_setup',
        output='screen',
        parameters=[{'namespaces': [d[0] for d in DRONES], 'rate_hz': 25.0}])

    return mavros + [streams]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mavproxy', default_value='false',
            description='true if SITL runs with MAVProxy forwarding to UDP 14551-14554; '
                        'false to connect straight to SITL TCP 5760-5790'),
        OpaqueFunction(function=launch_setup),
    ])
