from setuptools import find_packages, setup
from os import path
from glob import glob

package_name = 'bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (path.join('share', package_name, 'launch'), glob(path.join('launch', '*launch.[pxy][yma]*'))),
        (path.join('share', package_name, 'config'), glob(path.join('config', '*'))),
        (path.join('share', package_name, 'worlds'), glob(path.join('worlds', '*'))),
        (path.join('share', package_name, 'models', 'diff_drive'), glob(path.join("models", "diff_drive", "*"), recursive=True)),
        (path.join('share', package_name, 'models', 'diff_drive_ros'), glob(path.join("models", "diff_drive_ros", "*"), recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryan',
    maintainer_email='test@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
