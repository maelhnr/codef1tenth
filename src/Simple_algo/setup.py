from setuptools import setup
import os
from glob import glob

package_name = 'Simple_algo'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='etdisc',
    maintainer_email='etdisc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gap_follow = Simple_algo.follow_gap_penn:main',
            'wall_follow = Simple_algo.wall_follow:main',
            'wall_follow_penn = Simple_algo.wall_follow_penn:main',
            'disparity_extender = Simple_algo.DisparityExtender:main',
            'fast_disparity_extender = Simple_algo.fast_disparity_extender:main'
        ],
    },
)
