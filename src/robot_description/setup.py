import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_description'

def get_data_files(directory):
    data_files = []

    for root, dirs, files in os.walk(directory):
        if files:
            install_dir = os.path.join(
                'share',
                package_name,
                root
            )

            file_paths = [
                os.path.join(root, file)
                for file in files
            ]

            data_files.append(
                (install_dir, file_paths)
            )

    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),glob(os.path.join('launch', '*launch.*'))),
        (os.path.join('share', package_name, 'rviz'),glob(os.path.join('rviz', '*rviz'))),
    ] + get_data_files('urdf'),
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lihsun',
    maintainer_email='slps101023@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
