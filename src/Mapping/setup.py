from setuptools import setup
import os
from glob import glob
import os
from glob import glob

package_name = 'Mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'cartographer_config'), glob('cartographer_config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='etdisc',
    maintainer_email='nils.cahingt@student.isae-supaero.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy_grid = Mapping.Occupancy_Grid:main',
            'pose_listener = Mapping.pose_listener:main',
            'scan_filter = Mapping.scan_filter:main',
            'centerline_extraction = Mapping.centerline_extraction:main',
	    'custom_slam = Mapping.custom_slam:main',
        ],
    },
)
