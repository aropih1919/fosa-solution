from setuptools import find_packages, setup

package_name = 'caytu_perception'

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
    description='Détection AprilTag et obstacles hauts par profondeur.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'tag_detector = caytu_perception.detecteur_tags:main',
            'depth_scan = caytu_perception.noeud_scan_profondeur:main',
        ],
    },
)
