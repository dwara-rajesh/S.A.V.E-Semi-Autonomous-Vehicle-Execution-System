from setuptools import find_packages, setup

package_name = 'waypoint_follower_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pandas'],
    zip_safe=True,
    maintainer='dwarakesh',
    maintainer_email='dwarakesh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_follower = waypoint_follower_pkg.waypoint:main'
        ],
    },
)
