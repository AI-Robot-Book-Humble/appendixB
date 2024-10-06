from setuptools import find_packages, setup

package_name = 'airobot_action'

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
    maintainer='masutani',
    maintainer_email='masutani@isc.osakac.ac.jp',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bringme_action_client_node = airobot_action.bringme_action_client_node:main',
            'bringme_action_server_node = airobot_action.bringme_action_server_node:main',
            'new_bringme_action_server_node = airobot_action.new_bringme_action_server_node:main',
            'test_client = airobot_action.test_client:main',
            'test_server = airobot_action.test_server:main',
        ],
    },
)
