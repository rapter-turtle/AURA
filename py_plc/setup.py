from setuptools import find_packages, setup

package_name = 'py_plc'

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
    maintainer='jeyong',
    maintainer_email='jeyong@subak.io',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plc_comm = py_plc.plccomm:main',
            'plc_comm_test = py_plc.plccomm_test:main',
            'talker = py_plc.talk:main',
            'listener = py_plc.listen:main',
        ],
    },
)
