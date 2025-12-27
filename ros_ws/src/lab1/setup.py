from setuptools import find_packages, setup

# We're going to use these to install the launch files.
import os
from glob import glob

package_name = 'lab1'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
 
        # This line makes sure the launch files are installer.  This will copy
        # all the files in the launch directory to the install location.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='grimmc',
    maintainer_email='grimmc@oregonstate.edu',
    description='ROS 2 Go and Stop assignment',
    license='BSD-3-Clause',
    
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Hello world lab 1 node
            'lab1 = lab1.lab1:main',

            # Example of a twist command.
            'driver = lab1.driver:main',

            # Example of listening to a laser scan to do a stop
            'dumb_stopper = lab1.dumb_stopper:main',

            # YOUR CODE IN HERE - Go and stop when close
            'stopper = lab1.stopper:main',
        ],
    },
)
