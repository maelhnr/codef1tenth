import os
from glob import glob
from setuptools import setup

package_name = 'timer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nils',
    maintainer_email='nils@todo.todo',
    description='Package to evaluate the execution time of nodes or frequency of topics. Needs to be run with the ros2 run command.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "timer = timer.timer_node:main",
        ],
    },
)
