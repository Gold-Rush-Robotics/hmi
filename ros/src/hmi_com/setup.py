from setuptools import setup

package_name = 'hmi_com'

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
    maintainer_email='goldrushrobotics@gmail.com',
    description='This ROS package provides real-time insights into active ROS nodes by publishing JSON-formatted information about their publishers, subscribers, service servers, and service clients.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "start = hmi_com.hmi_com_main:main"
        ],
    },
)
