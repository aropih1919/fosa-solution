from setuptools import find_packages, setup

package_name = 'caytu_camera_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johny',
    maintainer_email='johny24.poseidon@gmail.com',
    description='Filtre hauteur du nuage top_camera + watchdog pret pour Nav2 (local, non pousse).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'height_filter_node = caytu_camera_perception.height_filter_node:main',
            'camera_perception_watchdog = caytu_camera_perception.camera_perception_watchdog:main',
        ],
    },
)
