from setuptools import find_packages, setup

package_name = 'swarm_nodes'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ricardo07',
    maintainer_email='ricardocango2007@gmail.com',
    description='Nodes for the SWARM-RSCL outdoor formation platform',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nodo_odometria = dron_control.nodo_odometria:main',
            'link_monitor = swarm_nodes.link_monitor:main',
            'first_flight = swarm_nodes.first_flight:main',
            'orientation_monitor = swarm_nodes.orientation_monitor:main',
            'stream_setup = swarm_nodes.stream_setup:main',
        ],
    },
)
