from setuptools import setup
import os
from glob import glob

package_name = 'strawberry_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.execution'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sun',
    maintainer_email='sun@todo.todo',
    description='Mock package for strawberry_motion',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
