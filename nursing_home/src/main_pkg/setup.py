from setuptools import find_packages, setup
import glob

package_name = 'main_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('lib/' + package_name + '/database/', glob.glob('database/*.*')),
        ('lib/' + package_name + '/tools/', glob.glob('tools/*.*')),
        ('lib/' + package_name + '/utils/', glob.glob('utils/*.*')),
        ('share/' + package_name + '/utils/', glob.glob('utils/config.ini')),
        ('share/' + package_name + '/map/', glob.glob('map/*.*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoh',
    maintainer_email='yun5.dev@gmail.com',
    description='package for multi robots control',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_controller = main_pkg.main_controller:main'
        ],
    },
)