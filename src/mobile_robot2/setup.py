import glob

from setuptools import find_packages, setup

package_name = 'mobile_robot2'

setup(
    name=package_name,
    version='2.0.0',
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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'debug = mobile_robot2.debug:main',
            'trials = mobile_robot2.shandong_trials_module:main'
        ],
    },
)
