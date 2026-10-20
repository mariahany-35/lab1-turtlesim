from setuptools import find_packages, setup

package_name = 'lab1_nodes'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maria',
    maintainer_email='maria@example.com',
    description='Lab 1 nodes: goToPose, reset, square',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'go_to_pose_server = lab1_nodes.go_to_pose_server:main',
            'reset_service = lab1_nodes.reset_service:main',
            'square_action_server = lab1_nodes.square_action_server:main',
        ],
    },
)
