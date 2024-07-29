import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'yolov5'
share_dir = 'share/' + package_name
lib_dir = 'lib/python3.10/site-packages/' + package_name + '/seg'

setup(
    name=package_name,
    version='0.1.0',
    # packages=[package_name, utils, models],
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        #(share_dir + '' )
        (lib_dir , glob.glob(os.path.join('data', '*.pt'))),
        (lib_dir ,['data/requirements.txt']),
        (lib_dir ,['data/setup.cfg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bkyb',
    maintainer_email='bkyb@todo.todo',
    description='Yolov7 and v8 package',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'predict_ui = yolov5.seg.segment.predict_ui:main',
        ],
    },
)
