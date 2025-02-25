from setuptools import setup

package_name = 'launch_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch_scripts/launch/control_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gene',
    maintainer_email='fgfg0203@g.skku.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
