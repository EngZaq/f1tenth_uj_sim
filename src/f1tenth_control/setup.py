from setuptools import find_packages, setup

package_name = 'f1tenth_control'

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
    maintainer='zaids_laptop',
    maintainer_email='zqaisi2004@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_publisher = f1tenth_control.path_publisher:main',
            'pure_pursuit = f1tenth_control.pure_pursuit:main',
        ],
    },
)
