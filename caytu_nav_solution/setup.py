from setuptools import find_packages, setup

package_name = 'caytu_nav_solution'

setup(
    name=package_name,
    version='0.3.0',
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
                'localisation roues + IMU ou slam_toolbox, cartographie manuelle '
                'et filtrage du lidar — équipe Fosa',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # Nom imposé par le règlement :
            #   ros2 run caytu_nav_solution task_solution.py
            'task_solution.py = caytu_nav_solution.task_solution:main',
            'fosa_benchmark = caytu_nav_solution.benchmark_cli:main',
            'fosa_compare = caytu_nav_solution.compare_cli:main',
            # Nœuds lancés par caytu_nav_bringup/launch/solution_bringup.launch.py
            'localisateur_odom_imu = caytu_nav_solution.localisateur_odom_imu:main',
            'filtre_sol_lidar = caytu_nav_solution.filtre_sol_lidar:main',
            # Cartographie manuelle (caytu_nav_bringup/launch/mapping.launch.py).
            'mapping_session = caytu_nav_solution.mapping_session:main',
            'save_map = caytu_nav_solution.save_map:main',
            'edit_map = caytu_nav_solution.edit_map:main',
            'tf_bag_relay = caytu_nav_solution.tf_bag_relay:main',
            # Nav2 attend la localisation (solution_bringup.launch.py).
            'wait_for_tf = caytu_nav_solution.wait_for_tf:main',
            # Course de la phase finale (caytu_nav_bringup/launch/course.launch.py).
            'course_tags = caytu_nav_solution.course_tags:main',
            'depart = caytu_nav_solution.depart:main',
            'diagnostic = caytu_nav_solution.diagnostic_course:main',
        ],
    },
)
