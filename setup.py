from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'turtlesim_move_extension_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools','pynput'],
    zip_safe=True,
    maintainer='miguel',
    maintainer_email='miguel.gonzalez72@alumnos.upm.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_control = turtlesim_move_extension_pkg.turtle_control:main',
        ],
    },
)
