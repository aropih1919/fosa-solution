from setuptools import find_packages, setup

package_name = 'caytu_nav_solution'

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
    maintainer='Liantsoa',
    maintainer_email='liantsoaandreane@gmail.com',
    description='Point d\'entrée officiel PARC2026 — Goal + Nav2 Client',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_solution = caytu_nav_solution.task_solution:main',
            # Installé afin que le bringup puisse le lancer depuis le package,
            # sans chemin source absolu ni `python3 <fichier>` manuel.
            'localization_watchdog = caytu_nav_solution.localization_watchdog:main',
        ],
    },
)
