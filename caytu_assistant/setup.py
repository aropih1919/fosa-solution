from glob import glob

from setuptools import find_packages, setup

package_name = 'caytu_assistant'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Équipe Fosa',
    maintainer_email='johny@fosa.mg',
    description='Assistant vocal CAYTU avec intention structurée et dispatcher sûr.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dialogue_manager = caytu_assistant.gestionnaire_dialogue:main',
            'speech_input = caytu_assistant.entree_vocale:main',
            'speech_output = caytu_assistant.sortie_vocale:main',
        ],
    },
)
