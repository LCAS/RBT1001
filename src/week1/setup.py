from setuptools import find_packages, setup
from os import path
from glob import glob

package_name = 'week1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (path.join('share', package_name, 'launch'), glob(path.join('launch', '*launch.[pxy][yma]*'))),
        (path.join('share', package_name, 'description', 'urdf'), glob(path.join('description', 'urdf', '*.xacro')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francesco Del Duchetto',
    maintainer_email='fdelduchetto@lincoln.ac.uk',
    description='CMP3103 teaching and assessment relevant package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtles_controller = week1.turtles_controller:main',
        ],
    },
)
