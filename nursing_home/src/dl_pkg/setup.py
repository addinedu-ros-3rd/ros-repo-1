from setuptools import find_packages, setup
import glob

package_name = 'dl_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models/', glob.glob('models/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haneol',
    maintainer_email='haneol0415@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_predictor = dl_pkg.action_predictor:main'
        ],
    },
)
