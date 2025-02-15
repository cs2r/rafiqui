from setuptools import find_packages, setup

package_name = 'rafiqui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['rafiqui/config.json']),
        ('share/' + package_name + '/launch', ['launch/start_robot.launch.py']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rafiqui',
    maintainer_email='rafiqui@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'ros_driver = rafiqui.ros_driver:main',
        ],
    },
)
