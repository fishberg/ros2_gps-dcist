from setuptools import find_packages, setup

package_name = 'gps_dcist'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gps_dcist.launch.yaml', 'launch/upenn_ntrip.launch.py', 'launch/ublox.launch.py']),
        ('share/' + package_name + '/config', ['config/c94_m8p_rover.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishberg',
    maintainer_email='fishberg.dev@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_monitor_node = gps_dcist.gps_monitor_node:main',
            'ntrip_monitor_node = gps_dcist.ntrip_monitor_node:main',
        ],
    },
)
