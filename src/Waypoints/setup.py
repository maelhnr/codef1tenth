from setuptools import setup

package_name = 'Waypoints'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'waypoints_logger = Waypoints.waypoint_logger_pure_pursuit:main',
            'pure_pursuit2 = Waypoints.pure_pursuit2:main',
            'pure_pursuit = Waypoints.pure_pursuit:main'
        ],
    },
)
