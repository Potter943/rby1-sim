from glob import glob

from setuptools import find_packages, setup


package_name = 'rby1_planner'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='RB-Y1 Developer',
    maintainer_email='user@example.com',
    description='Camera-aware autonomous frontend for the RB-Y1 backend.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner = rby1_planner.main:main',
        ],
    },
)
