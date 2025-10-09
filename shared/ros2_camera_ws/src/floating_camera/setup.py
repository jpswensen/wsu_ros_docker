# from setuptools import find_packages, setup

# package_name = 'camera_demo'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='me485',
#     maintainer_email='me485@todo.todo',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     extras_require={
#         'test': [
#             'pytest',
#         ],
#     },
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )


from setuptools import setup
import os
from glob import glob

package_name = 'floating_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'floating_camera'), glob('floating_camera/*.world')),
        (os.path.join('share', package_name, 'config'), glob('floating_camera/config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@todo.todo',
    description='Minimal Gazebo camera with entity state control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'spawn_camera.py = floating_camera.spawn_camera:main',
            'camera_demo.py = floating_camera.camera_demo:main'
        ],
    },
)
