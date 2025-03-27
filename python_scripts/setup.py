from setuptools import find_packages, setup

package_name = 'python_scripts'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ieeerobotics-pi',
    maintainer_email='zacharywerthman@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ICPtoOdom = python_scripts.ICPtoOdom:main',
	    'planner = python_scripts.planner:main',
	    'avoidance = python_scripts.avoidance:main',
        ],
    },
)
