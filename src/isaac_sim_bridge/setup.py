from setuptools import find_packages, setup

package_name = 'isaac_sim_bridge'

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
    maintainer='sun',
    maintainer_email='oceanshape700@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'isaac_sim_bridge_node = isaac_sim_bridge.isaac_sim_bridge_node:main',
            'test_move_node = isaac_sim_bridge.test_move_node:main',
        ],
    },
)
