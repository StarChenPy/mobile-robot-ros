
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_navigation_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']), 
    data_files=[
        # 必须的索引文件
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 包描述文件
        ('share/' + package_name, ['package.xml']),
        # 安装launch目录（如果有launch文件）
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 安装maps目录（如果有地图文件）
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        # 安装params目录（如果有参数文件）
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jxkj_mobile_robot_v1',
    maintainer_email='1602108136@qq.com',
    description='ROS 2 fixed navigation service package',
    license='Apache-2.0',  
    extras_require={ 
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'fixed_nav_service = my_navigation_py.fixed_nav_service:main',
            'cmd_vel_bridge = my_navigation_py.cmd_vel_bridge:main',
        ],
    },
)
