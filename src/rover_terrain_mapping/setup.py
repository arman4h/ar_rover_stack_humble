from setuptools import setup
package_name='rover_terrain_mapping'
setup(name=package_name, version='0.1.0', packages=[package_name], data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml'])], install_requires=['setuptools'], zip_safe=True, maintainer='Rover Team', maintainer_email='maintainer@example.com', description='Rolling terrain mapper', license='Apache-2.0', entry_points={'console_scripts':['terrain_mapper = rover_terrain_mapping.terrain_mapper:main']})
