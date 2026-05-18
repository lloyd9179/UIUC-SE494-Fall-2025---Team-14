#### SE 494, Fall 2025
### Written by Junyang Guan
###  lloyd9179@gmail.com
##### University of Illinois Urbana Champaign

from setuptools import find_packages, setup

package_name = 'path_exp'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='user@todo.todo',
    description='Package for UR robot path experiments.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spiral_test = path_exp.spiral_node:main',
            'spiral_real = path_exp.spiral_real:main',
            'traj_spiral = path_exp.cad_traj_spiral:main',
            'traj_rings = path_exp.cad_traj_rings:main',
            'traj_rings_completed = path_exp.traj_ring_completed:main',
            'test_gripper = path_exp.test_gripper:main',
           
        ],
    },
)