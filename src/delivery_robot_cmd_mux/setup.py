from setuptools import find_packages, setup

package_name = 'delivery_robot_cmd_mux'
setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cmd_mux.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tuan',
    maintainer_email='tuan@example.com',
    description='Command authority mux and watchdog for manual versus Nav2 velocity commands.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['cmd_mux = delivery_robot_cmd_mux.node:main']},
)
