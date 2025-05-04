from setuptools import setup
import os
from glob import glob

package_name = 'walle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'tf_transformations'],
    zip_safe=True,
    maintainer='danilo',
    maintainer_email='danilovieira@alu.ufc.br',
    description='Control logic for Walle robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = walle_control.obstacle_avoidance:main',
            'basic_movement = walle_control.basic_movement:main',
        ],
    },
)
