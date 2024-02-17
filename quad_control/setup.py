from setuptools import find_packages, setup

package_name = 'quad_control'

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
    maintainer='arc-hitter',
    maintainer_email='arrchit.jain@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trot_controller = quad_control.trot_controller:main',
            'leg_controller = quad_control.leg_controller:main',
            'creep_controller = quad_control.creep_controller:main',
            'custom_controller = quad_control.custom_controller:main',
        ],
    },
)
