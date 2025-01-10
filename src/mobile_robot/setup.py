from setuptools import find_packages, setup
from glob import glob

package_name = 'mobile_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 指定需要安装的文件夹
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools', 'rclpy', 'python-opencv', 'onnxruntime'],
    zip_safe=True,
    maintainer='starchen',
    maintainer_email='3425316708@qq.com',
    description='这是移动机器人节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_catch_fruit = mobile_robot.robot_catch_fruit:main'
        ],
    },
)
