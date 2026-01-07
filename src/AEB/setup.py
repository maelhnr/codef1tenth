from setuptools import setup
import os
from glob import glob

package_name = 'AEB'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='etdisc',
    maintainer_email='Pieter.Van-Holm@student.isae-supaero.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'AEB = AEB.AEB:main',
            'aeb2 = AEB.aeb2:main',
            'multiplexer = AEB.multiplexer:main'
        ],
    },
)
