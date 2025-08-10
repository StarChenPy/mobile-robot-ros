from setuptools import find_packages, setup

package_name = 'corn_robot_toolbox'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='starchen',
    maintainer_email='3425316708@qq.com',
    description='玉米机器人的工具包，不提供可执行文件',
    license='LGPL-3.0-only',
    entry_points={
        'console_scripts': [
        ],
    },
)
