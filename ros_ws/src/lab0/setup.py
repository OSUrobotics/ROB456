from setuptools import find_packages, setup

# We're going to use these to install the launch files.
import os
from glob import glob

package_name = 'lab0'

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
    description='ROS 2 examples for ROB456/514',
    license='BSD-3-Clause',
    
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # A basic publisher and subscriber.
            'publisher = lab0.publisher:main',
            'subscriber = lab0.subscriber:main',

            # The point circler
            'circler = lab0.circler:main',
        ],
    },
)
