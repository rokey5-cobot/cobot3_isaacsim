from setuptools import find_packages, setup

package_name = 'yolo_obb_3d'

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
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='3D OBB + Pose Estimation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'obb_node = yolo_obb_3d.obb_node:main',
            'obb_node_fin = yolo_obb_3d.obb_node_fin:main',
            'pose_subscriber_ros = yolo_obb_3d.pose_subscriber_ros:main',
        ],
    },
)

