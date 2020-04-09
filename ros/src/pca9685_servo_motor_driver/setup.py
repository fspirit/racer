from setuptools import setup

package_name = 'pca9685_servo_motor_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stanislav Olekhnovich',
    maintainer_email='stas.olekhnovich@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['pca9685_servo_motor_driver = pca9685_servo_motor_driver.pca9685_servo_motor_driver:main'
        ],
    },
)