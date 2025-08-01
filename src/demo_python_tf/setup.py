from setuptools import find_packages, setup

package_name = 'demo_python_tf'

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
    maintainer_email='hazhao@siue.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf_broadcaster=demo_python_tf.static_tf_broadcaster:main',
            'tf_listener=demo_python_tf.tf_listener:main',
            'moveit_client=demo_python_tf.moveit_FK:main',
            'moveit_client_IK=demo_python_tf.moveit_IK:main',
            'get_angles=demo_python_tf.get_angle:main'
        ],
    },
)
