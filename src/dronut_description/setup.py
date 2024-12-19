from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dronut_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include model files
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.[dae|stl]*'))),
        # Include URDF (.urdf) files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf.xacro*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.gazebo.xacro*'))),
        # Include rviz (.rviz) files
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nathan',
    maintainer_email='nathanshankar465@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)