from setuptools import find_packages, setup

package_name = 'pid_based_motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saeed',
    maintainer_email='saeedmridha42@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'check_data_subscription = pid_based_motor_control.check_data_subscription:main',
            'motor_speed_control_pid = pid_based_motor_control.motor_speed_control_pid:main',
            'test_right_motor = pid_based_motor_control.test_right_motor:main',
        ],
    },
)
