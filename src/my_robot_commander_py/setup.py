from setuptools import find_packages, setup

package_name = 'my_robot_commander_py'

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
    maintainer='viet',
    maintainer_email='viet92hcm@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_moveit = my_robot_commander_py.test_moveit:main',
            'commander = my_robot_commander_py.commander_template:main',
            'commander_humble = my_robot_commander_py.commander_template_humble:main'
        ],
    },
)
