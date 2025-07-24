from setuptools import setup

package_name = 'jazzy_bridge_pkg'

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
            'jazzy_scan_republisher = jazzy_bridge_pkg.jazzy_scan_republisher:main',
            'jazzy_tf_republisher = jazzy_bridge_pkg.jazzy_tf_republisher:main'
        ],
    },
)
