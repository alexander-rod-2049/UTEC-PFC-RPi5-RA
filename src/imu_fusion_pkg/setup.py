from setuptools import find_packages, setup

package_name = 'imu_fusion_pkg'

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
    maintainer='rmf209',
    maintainer_email='rmf209@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_reader_node = imu_fusion_pkg.imu_reader_node:main',
            'imu_fusion_node = imu_fusion_pkg.imu_fusion_node:main',
            'position_estimator_node = imu_fusion_pkg.position_estimator_node:main',
            'gps_node = imu_fusion_pkg.gps_node:main',
            'imu_node = imu_fusion_pkg.imu_node:main',
            'imu_filter_node = imu_fusion_pkg.imu_filter_node:main',
        ],
    },
)
