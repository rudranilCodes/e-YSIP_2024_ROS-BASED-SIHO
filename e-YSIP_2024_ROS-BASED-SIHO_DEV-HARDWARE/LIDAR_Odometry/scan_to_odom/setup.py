from setuptools import find_packages, setup
package_name = 'scan_to_odom'

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
    maintainer='aditya',
    maintainer_email='007bondadityaray@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    scripts=['scripts/Align2D.py'],
    entry_points={
        'console_scripts': [
        'visualise_odom = scan_to_odom.main:main'
        ],
    },
)
