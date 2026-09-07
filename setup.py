from setuptools import find_packages, setup

package_name = 'my_square_pkg'

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
    maintainer='sanrobo',
    maintainer_email='tomotomo1219y@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'square_move = my_square_pkg.square_move:main',
        'accel_deaccel = my_square_pkg.accel_deaccel:main',
        'accel_deaccel_2 = my_square_pkg.accel_deaccel_2:main',
        'accel_deaccel_3 = my_square_pkg.accel_deaccel_3:main',
        ],
    },
)
