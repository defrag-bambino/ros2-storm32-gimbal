from setuptools import setup

package_name = 'storm32_gimbal_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'tf_transformations'],
    zip_safe=True,
    maintainer='Fabian Domberg',
    maintainer_email='fabian.domberg@student.uni-luebeck.de',
    description='ROS2 SToRM32-Gimbal-Controller.',
    license='GNU General Public License v2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'storm32_gimbal_controller = storm32_gimbal_controller.storm32_controller_node:main'
        ],
    },
)
