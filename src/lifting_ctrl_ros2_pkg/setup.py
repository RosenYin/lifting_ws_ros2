from setuptools import find_packages, setup
import glob
import sys

package_name = 'lifting_ctrl_ros2_pkg'

config_files = glob.glob(package_name + '/config/*.json')

python_version = f'{sys.version_info.major}.{sys.version_info.minor}'
so_files = glob.glob(f'{package_name}/so/python{python_version}/*.so')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'lib/python{python_version}/site-packages/' + package_name + '/config', config_files),
        (f'lib/python{python_version}/site-packages/' + package_name , so_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_850pro = lifting_ctrl_ros2_pkg.service_node_850pro:main',
            'service_830abs = lifting_ctrl_ros2_pkg.service_node_830abs:main',
        ],
    },
)
