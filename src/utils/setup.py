from setuptools import find_packages, setup

package_name = 'utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/px4.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Damian Kryzia',
    maintainer_email='dkryzia@cpp.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'target_pose_publisher = utils.target_pose_publisher:main'
        ],
    },
)
