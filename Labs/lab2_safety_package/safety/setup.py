from setuptools import find_packages, setup

package_name = 'safety'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nadilgunawardane',
    maintainer_email='nadilgunawardane@todo.todo',
    description='TODO: Safety package for simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = safety.safety_node:main',
        ],
    },
)
