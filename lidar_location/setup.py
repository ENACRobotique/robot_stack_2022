from setuptools import setup

package_name = 'lidar_location'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Melvin Diez & Lilian Eve',
    maintainer_email='melvin.diez@free.fr',
    description='Uses data from lidar scans to get robots location on a stnadrardised Eurobot table.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'comm_node = lidar_location.topic_node:main'
        ],
    },
)
