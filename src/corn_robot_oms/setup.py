import glob

from setuptools import find_packages, setup

package_name = 'corn_robot_oms'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/param", glob.glob("param/*")),
        ('share/' + package_name + "/config", glob.glob("config/*")),
        ('share/' + package_name + "/launch", glob.glob("launch/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='starchen',
    maintainer_email='3425316708@qq.com',
    description='玉米的OMS系统',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pub_motions = corn_robot_oms.pub_motions:main"
        ],
    },
)
