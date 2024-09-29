from setuptools import setup

package_name = 'turtle_controller2'

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
    maintainer='macgyver',
    maintainer_email='dj@gagliardi.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_openLoop = turtle_controller2.turtle_openLoop:main',
            'turtle_openLoop2 = turtle_controller2.turtle_openLoop2:main'
        ],
    },
)
