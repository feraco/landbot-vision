from setuptools import setup
import os
from glob import glob

package_name = 'example_demos'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Demo Maintainer',
    maintainer_email='demo@example.com',
    description='Standalone demo scripts for ROS2 example package functionality',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'body_control_demo = example_demos.body_control_demo:main',
            'color_detect_demo = example_demos.color_detect_demo:main',
            'hand_gesture_demo = example_demos.hand_gesture_demo:main',
            'hand_track_demo = example_demos.hand_track_demo:main',
            'body_track_demo = example_demos.body_track_demo:main',
            'color_sorting_demo = example_demos.color_sorting_demo:main',
            'fall_detection_demo = example_demos.fall_detection_demo:main',
            'self_driving_demo = example_demos.self_driving_demo:main',
            'lane_following_demo = example_demos.lane_following_demo:main',
            'traffic_sign_demo = example_demos.traffic_sign_demo:main',
        ],
    },
)