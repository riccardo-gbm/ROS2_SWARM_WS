from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DRONES = [  # (namespace, sysid, esp_ip, laptop_port)
    ('drone1', 19, '192.168.50.11', 14551),  # PAPOI
    ('drone2', 17, '192.168.50.12', 14552),  # TRINEO
    ('drone3', 32, '192.168.50.13', 14553),  # BISUU
    ('drone4', 31, '192.168.50.14', 14554),  # LUCAS
]


def generate_launch_description():
    node_launch = PathJoinSubstitution([FindPackageShare('mavros'), 'launch', 'node.launch'])
    apm_config = PathJoinSubstitution([FindPackageShare('mavros'), 'launch', 'apm_config.yaml'])
    plugins = PathJoinSubstitution(
        [FindPackageShare('swarm_bringup'), 'config', 'swarm_pluginlists.yaml'])

    mavros = [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(node_launch),
            launch_arguments={
                'fcu_url': f'udp://:{port}@{ip}:14555',
                'gcs_url': '',
                'tgt_system': str(sysid),
                'tgt_component': '1',
                'pluginlists_yaml': plugins,
                'config_yaml': apm_config,
                'namespace': ns,
            }.items())
        for ns, sysid, ip, port in DRONES]

    streams = Node(
        package='swarm_nodes', executable='stream_setup', name='stream_setup',
        output='screen',
        parameters=[{'namespaces': [d[0] for d in DRONES], 'rate_hz': 25.0}])

    return LaunchDescription(mavros + [streams])