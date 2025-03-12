from setuptools import find_packages, setup
from glob import glob
import os
import sys

package_name = 'llm_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*')),
    ],
    install_requires=[
        'setuptools',
        'langchain',
        'langchain_community'
    ],    
    zip_safe=True,
    maintainer='reynaldy',
    maintainer_email='jawaechan@user.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate_waypoint = llm_perception.navigate_waypoint:main',
            'safety_navigation = llm_perception.safety_navigate:main',
        ],
    },
)
