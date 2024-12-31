from setuptools import find_packages, setup

package_name = 'robot_catch_fruit'

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
    maintainer='starchen',
    maintainer_email='3425316708@qq.com',
    description='这是移动机器人抓水果节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_catch_fruit = robot_catch_fruit.robot_catch_fruit:main'
        ],
    },
)
