import glob

from setuptools import setup

package_name = 'corn_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/param", glob.glob("param/*")),
        ('share/' + package_name + "/config", glob.glob("config/*")),
        ('share/' + package_name + "/launch", glob.glob("launch/*")),
        ('share/' + package_name + "/urdf", glob.glob("urdf/*")),
        ('share/' + package_name + "/maps", glob.glob("maps/*"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='starchen',
    maintainer_email='3425316708@qq.com',
    description='移动机器人项目功能包',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_waypoints = corn_robot.pub_waypoints:main',
            'generate_path = corn_robot.generate_path:main',
            'robot_motion = corn_robot.robot_motion:main',
            'navigation_to_waypoint = corn_robot.navigation_to_waypoint:main'
        ],
    },
)
