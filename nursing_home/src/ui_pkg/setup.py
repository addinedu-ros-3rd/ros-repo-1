from setuptools import find_packages, setup
import glob

package_name = 'ui_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/tools/', glob.glob('tools/*.*')),
        ('share/' + package_name + '/utils/', glob.glob('utils/*.*')),
        ('share/' + package_name + "/ui/", glob.glob('ui_pkg/monitoring.ui')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoh',
    maintainer_email='yun5.dev@gmail.com',
    description='GUI for manager(controlling multi robots)',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitoring = ui_pkg.monitoring:main'
        ],
    },
)
