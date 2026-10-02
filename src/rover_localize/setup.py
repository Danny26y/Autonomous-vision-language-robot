from setuptools import setup

package_name = 'rover_localize'

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
    description='Transforms detections into the map frame and publishes markers',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'target_locator = rover_localize.target_locator:main',
        ],
    },
)
