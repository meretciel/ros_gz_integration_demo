from setuptools import find_packages, setup
from os import path
from glob import glob
import os

package_name = 'bringup'

def scan_and_collect(dir_name, target_dir_prefix):
    full_path_of_root = path.join(path.dirname(path.abspath(__file__)), dir_name)
    base_dir_of_root = path.dirname(full_path_of_root)
    prefix_of_local_path = base_dir_of_root + "/"

    result = []
    for dirpath, dirnames, filenames in os.walk(full_path_of_root, followlinks=True):
        relative_dirpath = dirpath.replace(prefix_of_local_path, '', 1)
        files_at_this_level = [path.join(relative_dirpath, item) for item in filenames]
        new_path = dirpath.replace(base_dir_of_root, target_dir_prefix, 1)
        result.append((new_path, files_at_this_level))
    return result

model_files = scan_and_collect("models", path.join("share", package_name))

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
        *model_files
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
