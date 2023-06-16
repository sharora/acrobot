from setuptools import setup

package_name = 'acrobot'

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
    maintainer='shreyas',
    maintainer_email='sharora@gmail.com',
    description='Package to control hardware acrobot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'acrobot = acrobot.acrobot:main',
                'controller = acrobot.controller:main',
        ],
    },
)
