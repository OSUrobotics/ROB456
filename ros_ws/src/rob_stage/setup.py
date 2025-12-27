from setuptools import find_packages, setup

# We're going to use these to install the launch files.
import os
from glob import glob

package_name = 'rob_stage'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # This line makes sure the launch files are installed.  This will copy
        # all the files in the launch directory to the install location.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.yaml'))),
        (os.path.join('share', package_name, 'world'), glob(os.path.join('world', '*.world'))),
        (os.path.join('share', package_name, 'world/include'), glob(os.path.join('world/include', '*.inc'))),
        (os.path.join('share', package_name, 'world/bitmaps'), glob(os.path.join('world/bitmaps', '*.png'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cindygr',
    maintainer_email='grimmc@oregonstate.edu',
    description='Setup stage worlds for ROB456 and ROB 514',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
