import os
from glob import glob
from setuptools import setup

package_name = 'drone_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
        # (os.path.join('share', package_name), ['scripts/TerminatorScript.sh'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Braden',
    maintainer_email='braden@arkelectron.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'offboard_control = drone_sim.offboard_control:main',
                'visualizer = drone_sim.visualizer:main',
                'velocity_control = drone_sim.velocity_control:main',
                'control = drone_sim.control:main',
                'processes = drone_sim.processes:main',
                'dvs = drone_sim.dvs:main',
                'dvs_display = drone_sim.dvs_display:main'
        ],
    },
)
