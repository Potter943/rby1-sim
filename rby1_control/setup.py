from glob import glob
from setuptools import find_packages, setup

package_name = 'rby1_control'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RB-Y1 UI Developer',
    maintainer_email='user@example.com',
    description='RB-Y1 control backend with a Qt command-validation frontend.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_ui = rby1_control.main:main',
            'control_backend = rby1_control.backend_main:main',
        ],
    },
)
