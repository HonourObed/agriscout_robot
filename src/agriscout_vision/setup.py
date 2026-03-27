import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'agriscout_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # ADD THIS: This tells Python to include any .pt files inside the models folder
    package_data={
        package_name: ['models/*.pt'],
    },

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='honour-obed',
    maintainer_email='obedhonoureje@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detect_and_drive = agriscout_vision.detect_and_drive:main'
        ],
    },
)
