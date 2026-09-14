from setuptools import find_packages, setup

package_name = 'erc_libbot'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'rotate_90 = erc_libbot.rotate_90:main',
		'black_mask = erc_libbot.black_mask:main',
		'colour_mask = erc_libbot.colour_mask:main',
		'move_distance = erc_libbot.move_distance:main',
        ],
    },
)
