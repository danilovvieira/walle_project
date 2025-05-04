from setuptools import setup

package_name = 'wallerson_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danilo',
    maintainer_email='danilo@todo.todo',
    description='Pacote de controle do robô Wallerson',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basic_movement = wallerson_control.basic_movement:main',
        ],
    },
)

