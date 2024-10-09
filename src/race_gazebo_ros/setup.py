import os
from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = 'race_gazebo_ros'

# Copy all files from srcs to dst preserving the directory structure
def copy_all(dst, srcs):
    res = {}

    for src in srcs:
        for (path, directories, filenames) in os.walk(src):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join(dst, path)

                res[install_path] = [*res[install_path], file_path] if install_path in res else [file_path]

    return res.items()

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *copy_all('share/' + package_name, ['config/', 'launch/', 'models/', 'worlds/']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco Musto & Guilhem RICHAUD',
    maintainer_email='marco.musto@tuni.fi & guilhem.richaud@tuni.fi',
    description='Race simulator: Press \'s\' to start. A picture will be taken when the first car reaches the line.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start_node = race_gazebo_ros.start_node:main',
            'race_director = race_gazebo_ros.race_director:main',
        ],
    },
)