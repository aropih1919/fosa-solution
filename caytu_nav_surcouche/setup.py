from setuptools import find_packages, setup

package_name = 'caytu_nav_surcouche'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/surcouche.launch.py']),
        ('share/' + package_name + '/config', ['config/collision_monitor_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johny',
    maintainer_email='johny24.poseidon@gmail.com',
    description='Surcouche Nav2 : collision_monitor + inflation adaptative (local, non pousse).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'adaptive_inflation = caytu_nav_surcouche.adaptive_inflation:main',
        ],
    },
)
