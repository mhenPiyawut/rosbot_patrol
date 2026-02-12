from setuptools import find_packages, setup

package_name = 'patrol_logger_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mhenpiyawut',
    maintainer_email='piyawut.mm@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_logger_py_node = patrol_logger_py.patrol_logger_py_node:main'
        ],
    },
)
