from setuptools import find_packages, setup

package_name = 'caytu_semantic_map'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Équipe Fosa',
    maintainer_email='johny@fosa.mg',
    description='Carte sémantique et résolution de lieux ROS 2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'semantic_map_node = caytu_semantic_map.semantic_map_node:main',
        ],
    },
)
