from setuptools import setup

package_name = 'srv_examples'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlesim_geofence_reset_srv = srv_examples.turtlesim_geofence_reset_srv:main',
            'open_gripper_service_server = srv_examples.open_gripper_service_server:main',
            'invert_gripper_service_client = srv_examples.invert_gripper_service_client:main',
        ],
    },
)
