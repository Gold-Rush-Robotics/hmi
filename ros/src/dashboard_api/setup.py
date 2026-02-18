from setuptools import setup

package_name = 'dashboard_api'

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
    maintainer='GoldRushRobotics',
    maintainer_email='goldrushrobotics@charlotte.edu',
    description='A ROS 2 API for publishing structured dashboard data as JSON to a web-based HMI, allowing nodes to send cards, bars, and color indicators.',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
