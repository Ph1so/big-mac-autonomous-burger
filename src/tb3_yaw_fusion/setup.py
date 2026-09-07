from setuptools import find_packages, setup

package_name = 'tb3_yaw_fusion'

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
    maintainer='t103',
    maintainer_email='t103@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
            'console_scripts': [
                    'imu_listener = tb3_yaw_fusion.imu_yaw_integrator:main',
                    'odom_listener = tb3_yaw_fusion.odom_yaw_extractor:main',
            ],
    },
)
