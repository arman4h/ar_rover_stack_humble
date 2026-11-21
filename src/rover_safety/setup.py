from setuptools import setup
p='rover_safety'
setup(name=p,version='0.1.0',packages=[p],data_files=[('share/ament_index/resource_index/packages',['resource/'+p]),('share/'+p,['package.xml'])],install_requires=['setuptools'],zip_safe=True,maintainer='Rover Team',maintainer_email='maintainer@example.com',description='Velocity watchdog',license='Apache-2.0',entry_points={'console_scripts':['cmd_vel_watchdog = rover_safety.watchdog:main']})
