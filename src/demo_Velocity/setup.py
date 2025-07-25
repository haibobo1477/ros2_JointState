from setuptools import find_packages, setup

package_name = 'demo_Velocity'

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
    maintainer='haibo',
    maintainer_email='zhaohaibo177417@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jacobian_node=demo_Velocity.robot_jac:main',
            'dance_node=demo_Velocity.test_traj:main'
        ],
    },
)
