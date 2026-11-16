from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'visual_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lihsun',
    maintainer_email='slps101023@gmail.com',
    description='Python tools for XLeRobot visual SLAM integration.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'input_status = visual_slam.input_status:main',
            'route_sbdcb = visual_slam.route_sbdcb:main',
            'head_pose_commander = visual_slam.head_pose_commander:main',
        ],
    },
)
