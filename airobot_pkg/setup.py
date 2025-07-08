from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'airobot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    # 添加launch文件安装配置
    (os.path.join('share', package_name, 'launch'), 
        glob('launch/*.launch.py')),
    # 如果包含其他资源文件（如rviz配置、地图等）
    (os.path.join('share', package_name, 'config'), 
        glob('config/*')),
    (os.path.join('share', package_name, 'rviz'), 
        glob('rviz/*.rviz')),
    (os.path.join('share', package_name, 'maps'), 
        glob('maps/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunji',
    maintainer_email='sunji@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move=airobot_pkg.move:main',
            'aimove=airobot_pkg.aimove:main',
            'mbot_teleop=airobot_pkg.mbot_teleop:main',
            'smallcar_move=airobot_pkg.smallcar_move:main',
        ],
    },
)
