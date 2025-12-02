from setuptools import find_packages, setup

package_name = 'tile_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', [
            'worlds/cave.world',
            'worlds/light.world',
            'worlds/dark.world',
            'worlds/code.world',
            'worlds/windy.world'
        ]),
        ('share/' + package_name + '/launch', [
            'launch/p4.launch.py'
        ]),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ehtom',
    maintainer_email='smlamothe@tamu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_publisher = tile_localization.map_publisher:main',
            'localizer = tile_localization.localizer:main',
        ],
    },
)
