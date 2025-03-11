from setuptools import find_packages, setup
from os import path
from glob import glob

package_name = 'utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # (path.join('share', package_name, 'launch'), glob(path.join('launch', '*launch.[pxy][yma]*'))),
        # (path.join('share', package_name, 'description', 'urdf'), glob(path.join('description', 'urdf', '*.xacro'))),
        # (path.join('share', package_name, 'description', 'meshes', 'visual'), glob(path.join('description', 'meshes', 'visual', '*'))),
        (path.join('share', package_name, 'scripts'), glob(path.join('scripts', '*.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francesco Del Duchetto',
    maintainer_email='fdelduchetto@lincoln.ac.uk',
    description='RBT1001 utils package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_marker = scripts.simple_marker:main',
            'target_marker = scripts.target_marker:main'
        ],
    },
)