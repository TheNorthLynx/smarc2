from setuptools import find_packages, setup

package_name = 'evolo_activate_ghost'

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
    maintainer='Mattias',
    maintainer_email='mtrende@kth.se',
    description='Activates ghost obstacles for Evolo',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'evolo_activate_ghost_server = evolo_activate_ghost.evolo_activate_ghost_server:main'
        ],
    },
)
