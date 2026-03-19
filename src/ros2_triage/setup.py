from setuptools import setup, find_packages

package_name = 'ros2_triage'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # Include non-.py files bundled inside the package
    package_data={
        package_name: [
            'tui/theme.tcss',
        ],
    },
    install_requires=[
        'setuptools',
        'rich',
        'psutil',
        'pyyaml',
        'textual>=0.50.0',
        'pydantic>=2.0.0',
        'networkx>=3.0',
    ],
    zip_safe=True,
    maintainer='darshan',
    maintainer_email='user@example.com',
    description='ROS 2 CLI plugin for runtime graph diagnostics (v2: TUI + health engine)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'triage = ros2_triage.command.triage:TriageCommand',
        ],
        'console_scripts': [],
    },
)
