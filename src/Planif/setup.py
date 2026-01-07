from setuptools import setup
import os
from glob import glob

package_name = 'Planif'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
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
            'A_star = Planif.A_etoile:main',
            'A_star_SLAM = Planif.A_etoile_SLAM:main',
        ],
    },
)
