from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tod_carla_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jordan Kuruvilla',
    maintainer_email='jordankuruvilla2@gmail.com',
    description='TUM FTM vehicle interface stub for CARLA simulation.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
