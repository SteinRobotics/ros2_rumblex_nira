import os
from setuptools import find_packages, setup


package_name = 'nira_hmi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/hmi_launch.py', 'launch/hmi_simulated_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='christian',
    maintainer_email='stein.robotics@gmail.com',
    description='Human-machine interface (OLED display, relay control, power monitoring) for the Nira hexapod',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'node_hmi = nira_hmi.node_hmi:main',
            'node_hmi_simulated = nira_hmi.node_hmi_simulated:main',
        ],
    },
)
