import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'ur_chess'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools','ur_chess_msgs'],
    zip_safe=True,
    maintainer='appuser',
    maintainer_email='bancsimark02@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_controller = ur_chess.move_controller:main',
            'game_manager = ur_chess.game_manager:main',
        ],
    },
)
