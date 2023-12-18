from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_app'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib/python3.10/site-packages', package_name, 'signs_to_detect'), glob(os.path.join('signs_to_detect', '*.png'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ilyusha',
    maintainer_email='Syrenny@github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'signs_detection = robot_app.signs_detection:main',
        ],
    },
)
