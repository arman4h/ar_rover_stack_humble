from setuptools import setup
from glob import glob
import os
p='rover_bringup'
setup(name=p,version='0.1.0',packages=[p],data_files=[('share/ament_index/resource_index/packages',['resource/'+p]),('share/'+p,['package.xml']), (os.path.join('share',p,'launch'),glob('launch/*.launch.py')), (os.path.join('share',p,'config'),glob('config/*.yaml'))],install_requires=['setuptools'],zip_safe=True,maintainer='Rover Team',maintainer_email='maintainer@example.com',description='Rover bringup',license='Apache-2.0')
