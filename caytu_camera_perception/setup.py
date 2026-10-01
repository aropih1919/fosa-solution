from setuptools import find_packages, setup

package_name = 'caytu_camera_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/camera_obstacles.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Trinôme OpenCV & Caméra',
    maintainer_email='team@fosa.mg',
    description='Obstacles en hauteur (invisibles au LiDAR) à partir de la top_camera',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_obstacle_node = caytu_camera_perception.camera_obstacle_node:main',
            'camera_perception_watchdog = caytu_camera_perception.camera_perception_watchdog:main',
        ],
    },
)
