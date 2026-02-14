import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'eecs256aw25'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kjames',
    maintainer_email='kjames@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = eecs256aw25.publisher:publisher_entry_point_function',
            'listener = eecs256aw25.subscriber:subscriber_entry_point_function',
            'open_loop = eecs256aw25.open_loop:open_loop_entry_point_function',
            'closed_loop = eecs256aw25.closed_loop:closed_loop_entry_point_function',
        ],
    },
)
