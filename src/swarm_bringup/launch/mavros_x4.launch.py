from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

DRONES = [  # (namespace, sysid, esp_ip, puerto_laptop)
    ('drone1', 19, '192.168.50.11', 14551),  # PAPOI
    ('drone2', 17, '192.168.50.12', 14552),  # TRINEO
    ('drone3', 32, '192.168.50.13', 14553),  # BISUU
    ('drone4', 31, '192.168.50.14', 14554),  # LUCAS
]


def generate_launch_description():
    apm = PathJoinSubstitution([FindPackageShare('mavros'), 'launch', 'apm.launch'])
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(apm),
            launch_arguments={'fcu_url': f'udp://:{port}@{ip}:14555',
                              'tgt_system': str(sysid),
                              'namespace': ns}.items())
        for ns, sysid, ip, port in DRONES])