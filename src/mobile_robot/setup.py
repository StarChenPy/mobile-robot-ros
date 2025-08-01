import glob

from setuptools import find_packages, setup

package_name = 'mobile_robot'

setup(
    name=package_name,
    version='1.9.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/config", glob.glob("config/*")),
        ('share/' + package_name + "/param", glob.glob("param/*")),
        ('share/' + package_name + "/weights", glob.glob("weights/*"))
    ],
    install_requires=['setuptools', 'rclpy', 'python-opencv', 'onnxruntime'],
    zip_safe=True,
    maintainer='starchen',
    maintainer_email='3425316708@qq.com',
    description='这是移动机器人节点',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'test = mobile_robot.test_module:main',
            'b = mobile_robot.task_b_module:main',
            'd = mobile_robot.task_d_module:main',
            'e = mobile_robot.task_e_module:main',
            'debug = mobile_robot.debug_module:main'
        ],
    },
)
