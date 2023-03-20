from glob import glob
from setuptools import setup
import os

package_name = 'mocha_buono'
submodules = "mocha_buono/sensor"

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))

#launch files
data_files.append((os.path.join('share', package_name), glob('launch/*.py')))
#configs
data_files.append(('share/' + package_name + '/config', glob('config/*')))
#params
data_files.append(('share/' + package_name + '/params', glob('params/*')))


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, submodules],
    # py_modules=[
    #     package_name + '.sensor',
    # ],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bresilla',
    maintainer_email='trim.bresilla@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mocha_buono = mocha_buono.mocha_buono:main',
            'visualize = mocha_buono.visualize_imu:main'
        ],
    },
)
