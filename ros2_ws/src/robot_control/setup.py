import os
from glob import glob
from setuptools import find_packages, setup
from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matias Molina',
    maintainer_email='matiming123@gmail.com',
    description='Controlador industrial, UI y planificador para Robot 3 DOF',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_ui_node = robot_control.gui.robot_ui_node:main',
            'orchestrator_node = robot_control.orchestrator.publisher:main',
            'vision_detector_node = robot_control.vision.vision_detector_node:main',
            
        ],
    },
)