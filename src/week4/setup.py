from setuptools import find_packages, setup
from os import path
from glob import glob

package_name = 'week4'
submodules = ["scripts"]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=submodules),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (path.join('share', package_name, 'launch'), glob(path.join('launch', '*launch.[pxy][yma]*'))),
        # (path.join('share', package_name, 'description', 'urdf'), glob(path.join('description', 'urdf', '*.xacro'))),
        # (path.join('share', package_name, 'description', 'meshes', 'visual'), glob(path.join('description', 'meshes', 'visual', '*'))),
        (path.join('share', package_name, 'scripts'), glob(path.join('scripts', '*.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marc Hanheide',
    maintainer_email='marc@hanheide.net',
    description='CMP3103 teaching and assessment relevant package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward_kinematics_node = scripts.forward_kinematics_node:main',
            'inverse_kinematics_node = scripts.inverse_kinematics_node:main',
        ]
    }
)
