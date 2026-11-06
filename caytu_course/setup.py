from glob import glob

from setuptools import find_packages, setup

package_name = 'caytu_course'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Équipe Fosa',
    maintainer_email='johny@fosa.mg',
    description='Course de la phase réelle PARC 2026 : ordre des tags, '
                'départ (commande ou voix), recherche, visites et statut.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'course_parc = caytu_course.noeud_course:main',
            'garde_vitesse = caytu_course.noeud_garde_vitesse:main',
            'mot_depart = caytu_course.noeud_mot_depart:main',
            'suivi_course = caytu_course.suivi:main',
        ],
    },
)
