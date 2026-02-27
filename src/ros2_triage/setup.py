from setuptools import setup, find_packages

package_name = 'ros2_triage'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'colorama'],
    zip_safe=True,
    maintainer='darshan',
    maintainer_email='user@example.com',
    description='ROS 2 CLI plugin for runtime graph diagnostics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'triage = ros2_triage.command.triage:TriageCommand',
        ],
        'console_scripts': [],
    },
)
