from setuptools import find_packages, setup

package_name = 'my_examples'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={     
        'console_scripts': [
            'move_joint = my_examples.move_joint:main',
            'move_joint_fin = my_examples.move_joint_fin:main',
        ],
    },
)

