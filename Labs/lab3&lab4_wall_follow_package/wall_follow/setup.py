from setuptools import setup

package_name = 'wall_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nadilgunawardane',
    maintainer_email='nadilgunawardane@example.com',
    description='Wall following ROS 2 package with multiple nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follow_node = wall_follow.wall_follow_node:main',  # Existing wall-following node
            'car_drive_node = wall_follow.car_drive_node:main',      # New car drive node
            'reactive_node = wall_follow.reactive_node:main' # Lab3 template for follow the gap
        ],
    },
)
