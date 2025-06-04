from setuptools import find_packages, setup

package_name = 'control_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
        ('share/' + package_name + '/config',
            ['config/slam_params_base_link.yaml']),
        ('share/' + package_name + '/config',
            ['config/nav2_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='garjones',
    maintainer_email='garjones@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = control_node.control:main',
            'keyboard = control_node.keyboard:main',
            'explorer = control_node.explorer:main',
        ],
    },
)
