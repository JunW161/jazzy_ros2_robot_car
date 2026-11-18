from setuptools import find_packages, setup
package_name='delivery_robot_web_bridge'
setup(name=package_name, version='1.0.0', packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml']),('share/'+package_name+'/launch',['launch/web_bridge.launch.py'])],
 install_requires=['setuptools'], zip_safe=True, maintainer='tuan', maintainer_email='tuan@example.com',
 description='ROS bridge for later Web Nav2 visualization and goal control.', license='Apache-2.0',
 entry_points={'console_scripts':['web_bridge = delivery_robot_web_bridge.bridge:main']})
