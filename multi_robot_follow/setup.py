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
    maintainer='user',
    maintainer_email='your_github_email_here',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'triangle_formation = multi_robot_follow.triangle_formation:main',
        'displacement_formation = multi_robot_follow.displacement_formation:main',
        'collision_formation = multi_robot_follow.collision_formation:main',
    ],
},

)
