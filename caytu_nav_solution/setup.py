from setuptools import find_packages, setup

package_name = 'caytu_nav_solution'

setup(
    name=package_name,
    version='0.2.0',
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
    description='Point d\'entrée officiel PARC 2026 (task_solution.py), '
                'localisation roues + IMU et filtrage du lidar — équipe Fosa',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # Nom imposé par le règlement :
            #   ros2 run caytu_nav_solution task_solution.py
            'task_solution.py = caytu_nav_solution.task_solution:main',
            # Alias conservé pour les habitudes de l'équipe.
            'task_solution = caytu_nav_solution.task_solution:main',
            # Nœuds lancés par caytu_nav_bringup/launch/solution_bringup.launch.py
            'localisateur_odom_imu = caytu_nav_solution.odom_imu_localizer:main',
            'filtre_sol_lidar = caytu_nav_solution.lidar_floor_filter:main',
            # Anciens noms conservés pour ne pas casser les habitudes.
            'odom_imu_localizer = caytu_nav_solution.odom_imu_localizer:main',
            'lidar_floor_filter = caytu_nav_solution.lidar_floor_filter:main',
        ],
    },
)
