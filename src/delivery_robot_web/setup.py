from setuptools import setup, find_packages
setup(
    name='delivery_robot_web', version='6.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/delivery_robot_web']),
                ('share/delivery_robot_web', ['package.xml'])],
    package_data={'delivery_robot_web': ['static/*', 'launch/*.py', 'rviz/*.rviz', '*.sh', 'install_profile.json', 'source_contract.json']},
    install_requires=['setuptools'], zip_safe=False,
    maintainer='Robot workspace operator', maintainer_email='robot@example.invalid',
    description='Local delivery operations console with mapping, Nav2 lifecycle and mission control.',
    license='MIT',
    entry_points={'console_scripts': ['web = delivery_robot_web.cli:main']},
)
