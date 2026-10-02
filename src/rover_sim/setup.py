import os
from glob import glob
from setuptools import setup

package_name = 'rover_sim'

def files(pattern):
    return [f for f in glob(pattern) if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', files('launch/*.py')),
        ('share/' + package_name + '/worlds', files('worlds/*')),
        ('share/' + package_name + '/params', files('params/*.yaml')),
        ('share/' + package_name + '/models/turtlebot3_waffle', files('models/turtlebot3_waffle/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover',
    maintainer_email='rover@example.com',
    description='Simulation assets for the semantic navigation rover',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
