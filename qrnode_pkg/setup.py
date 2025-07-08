from setuptools import find_packages, setup

package_name = 'qrnode_pkg'

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
    maintainer='sunji',
    maintainer_email='sunji@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'qrnode=qrnode_pkg.qrnode:main',
            'number=qrnode_pkg.number:main',
            'post_publish=qrnode_pkg.post_publish:main',
            'all_qr_num=qrnode_pkg.all_qr_num:main',
            'myyolov8=qrnode_pkg.myyolov8:main',
            'rm_follow=qrnode_pkg.rm_follow:main',
            'follow=qrnode_pkg.follow:main',
            'rm_waypoint_navigation=qrnode_pkg.rm_waypoint_navigation:main',
            'rdk_power_inentification=qrnode_pkg.rdk_power_inentification:main',
            'rdk_power_QT=qrnode_pkg.rdk_power_QT:main',
            'rdk_power_QT_LLM=qrnode_pkg.rdk_power_QT_LLM:main',
        ],
    },
)
