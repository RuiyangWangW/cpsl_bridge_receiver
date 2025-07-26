from setuptools import setup

package_name = 'ros2_receiver_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Republish JSON socket messages into ROS Jazzy',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scan_receiver = ros2_receiver_pkg.scan_receiver:main',
            'tf_receiver = ros2_receiver_pkg.tf_receiver:main'
        ],
    },
)
