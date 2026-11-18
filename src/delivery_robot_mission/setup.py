from setuptools import find_packages, setup
package_name = 'delivery_robot_mission'
setup(
    name=package_name, version='1.0.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mission_manager.launch.py']),
        ('share/' + package_name + '/config', ['config/mission_example.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='tuan', maintainer_email='tuan@example.com',
    description='Pickup to dropoff mission state machine on top of Nav2 NavigateToPose.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'mission_manager = delivery_robot_mission.mission_manager:main',
        'mission_cli = delivery_robot_mission.mission_cli:main',
        'mission_from_web_points = delivery_robot_mission.mission_from_web_points:main',
        'mission_status = delivery_robot_mission.mission_status:main',
    ]},
)
