from setuptools import setup
import os
from glob import glob

package_name = 'orbit_bot_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # ADD THIS LINE BELOW:
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='draco',
    maintainer_email='user@todo.todo',
    description='Base driver for Orbit Bot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_driver = orbit_bot_base.base_driver:main'
        ],
    },
)
