from setuptools import find_packages, setup

package_name = 'multi_robot_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Apurva Palaniappan',
    maintainer_email='apurva.p2024@gmail.com',
    description='Cooperative Multi-Robot Navigation for TurtleBot3 using ROS 2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'displacement_formation = multi_robot_follow.displacement_formation:main',
        'collision_avoidance_only = multi_robot_follow.collision_avoidance_only:main',
        'connectivity_maintenance_only = multi_robot_follow.connectivity_maintenance_only:main',
        'integrated_multi_robot = multi_robot_follow.integrated_multi_robot:main',
        
        
    ],
},

)
