import glob
from setuptools import find_packages, setup

package_name = 'go2_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        ('share/' + package_name + '/launch',
            ['launch/go2.launch.py']),
        
        ('share/' + package_name + '/launch',
            ['launch/real.launch.py']),

        ('share/' + package_name + '/launch',
            ['launch/sim.launch.py']),

        ('share/' + package_name + '/launch',
            ['launch/mapping.launch.py']),
        ('share/' + package_name + '/config',
            glob.glob('config/*')),
        ('share/' + package_name + '/urdf',
            ['urdf/robot.urdf']),
        ('share/' + package_name + '/config',
            glob.glob('config/*')),
        ('share/' + package_name + '/meshes',
            glob.glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usys',
    maintainer_email='usys@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'send_goal = go2_control.send_goal:main',
        'waypoint_gen = go2_control.waypoint_gen:main',
        'init_pose = go2_control.init_pose:main',
        'main = go2_control.main:main',
        'cmd_vel_node = go2_control.cmd_vel_node:main',
        'go2w_read = go2_control.go2w_read:main',
        'Stream = go2_control.Stream:main',
        'camera = go2_control.camera:main',
        'fake_robot = go2_control.fake_robot:main',
        'gazebo_convert = go2_control.gazebo_convert:main',
        'april_localizer = go2_control.april_localizer:main',
        ],
    },
)
