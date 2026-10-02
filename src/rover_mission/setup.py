from setuptools import setup

package_name = 'rover_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover',
    maintainer_email='rover@example.com',
    description='Mission controller: approach a detected target through Nav2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission_controller = rover_mission.mission_controller:main',
        ],
    },
)
