from setuptools import find_packages, setup
import glob

package_name = 'robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/map/', glob.glob('map/*.*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoh',
    maintainer_email='yun5.dev@gmail.com',
    description='package for each robot',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = robot_pkg.robot_controller:main'
        ],
    },
)
