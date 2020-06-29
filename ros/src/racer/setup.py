import os
from setuptools import setup
from glob import glob

package_name = 'racer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fs',
    maintainer_email='stas.olekhnovich@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['servo_driver = racer.servo_driver.main:main',
            'control_error_extractor = racer.control_error_extractor.main:main'
        ],
    },
)
