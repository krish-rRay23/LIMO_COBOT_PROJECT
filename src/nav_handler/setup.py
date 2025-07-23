from setuptools import setup

package_name = 'nav_handler'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agilex',
    maintainer_email='raykrish25@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_handler = nav_handler.object_handler:main',
            'exploration_handler = nav_handler.exploration_handler:main',
            'pose_setter = nav_handler.pose_setter:main',
            'mission_manager = nav_handler.mission_manager:main',
        ],
    },
)
