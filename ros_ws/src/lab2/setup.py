from setuptools import find_packages, setup

# We're going to use these to install the launch files.
import os
from glob import glob

package_name = 'lab2'

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
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='grimmc',
    maintainer_email='grimmc@oregonstate.edu',
    description='ROS 2 Go around assignment',
    license='BSD-3-Clause',
    
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Hello world lab 2 node
            'lab2 = lab2.lab2:main',

            # YOUR CODE IN HERE Driver code (make the robot move)
            'driver = lab2.driver:main',

            # Send the robot a list of target points to go to
            'send_points = lab2.send_points:main',

        ],
    },
)
