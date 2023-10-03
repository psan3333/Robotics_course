from setuptools import find_packages, setup

package_name = 'service_full_name'

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
    maintainer='alexey',
    maintainer_email='psan3333@mail.ru',
    description='Simple service-client for passing name of the person',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_name = service_full_name.service:main',
            'client_name = service_full_name.client:main',
        ],
    },
)
