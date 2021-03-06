from setuptools import setup
import os
from glob import glob

package_name = 'aruco_analysis_enac'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonathan M',
    maintainer_email='jonathan_bleu@hotmail.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_aruco = aruco_analysis_enac.detect_aruco:main',
            'analysis_aruco = aruco_analysis_enac.analysis_aruco:main',
            'camera_calibrator = aruco_analysis_enac.camera_calibrator:main',
        ],
    },
)
