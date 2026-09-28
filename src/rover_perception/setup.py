from setuptools import setup

package_name = 'rover_perception'

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
    description='Perception node: detect targets and back-project to 3D',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'perception_node = rover_perception.perception_node:main',
        ],
    },
)
