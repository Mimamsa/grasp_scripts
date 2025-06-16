from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'grasp_scripts'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch/*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuhsienc',
    maintainer_email='illusion.dark@gmail.com',
    description='This ROS2 node is designated as a script, which triggered by user then executes commands step by step.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_and_place = grasp_scripts.grasp_and_place:main'
        ],
    },
)
