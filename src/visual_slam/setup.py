from setuptools import find_packages, setup

package_name = 'visual_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
