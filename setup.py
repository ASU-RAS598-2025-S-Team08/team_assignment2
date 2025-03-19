from setuptools import find_packages, setup

package_name = 'team_assignment2'

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
    maintainer='bhavya-shah',
    maintainer_email='bhavya-shah@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_viewer = team_assignment2.camera_viewer:main',
            'depth_viewer = team_assignment2.depth_viewer:main',
            'color_selector = team_assignment2.color_selector:main',
            'color_detector = team_assignment2.color_detector:main',
        ],
    },
)
