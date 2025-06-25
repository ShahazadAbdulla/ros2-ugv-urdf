from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ugv_description' # Package name

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Install all rviz config files
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        # Install all URDF and XACRO files, preserving directory structure
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.*'))),
        (os.path.join('share', package_name, 'urdf', 'mech'), glob(os.path.join('urdf', 'mech', '*.*'))),
        (os.path.join('share', package_name, 'urdf', 'sensors'), glob(os.path.join('urdf', 'sensors', '*.*'))),
        (os.path.join('share', package_name, 'urdf', 'robots'), glob(os.path.join('urdf', 'robots', '*.*'))),
        # Install mesh files, preserving directory structure under 'meshes'
        (os.path.join('share', package_name, 'meshes', 'rosmaster_x3', 'visual'), glob(os.path.join('meshes', 'rosmaster_x3', 'visual', '*.*'))),
        (os.path.join('share', package_name, 'meshes', 'intel_realsense', 'visual'), glob(os.path.join('meshes', 'intel_realsense', 'visual', '*.*'))),
        (os.path.join('share', package_name, 'meshes', 'hp60c', 'visual'), glob(os.path.join('meshes', 'hp60c', 'visual', '*.*'))),
        (os.path.join('share', package_name, 'meshes', 'pan_tilt'), glob(os.path.join('meshes', 'pan_tilt', '*.*'))),
        (os.path.join('share', package_name, 'meshes', 'IMU'), glob(os.path.join('meshes', 'IMU', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='UGV Robot Description Package',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)