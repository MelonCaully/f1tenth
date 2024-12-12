from setuptools import setup

package_name = 'pure_pursuit_pkg'

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
    maintainer='md0174',
    maintainer_email='md0174@mix.wvu.edu',
    description='Pure pursuit',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = pure_pursuit_pkg.pure_pursuit:main'
        ],
    },
)
