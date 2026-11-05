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
    description='AprilTag, profondeur géométrique, YOLO et suivi personne.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tag_detector = caytu_perception.detecteur_tags:main',
            'registre_tags = caytu_perception.registre_tags:main',
            'essai_tags = caytu_perception.essai_tags:main',
            'tag_calibrator = caytu_perception.calibrateur_tags:main',
            'obstacle_classifier = caytu_perception.classificateur_obstacles:main',
            'person_tracker = caytu_perception.suivi_personnes:main',
        ],
    },
)
