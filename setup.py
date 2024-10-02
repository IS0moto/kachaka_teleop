from setuptools import find_packages, setup

package_name = 'kachaka_teleop'

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
    maintainer='isomoto',
    maintainer_email='isomoto@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = kachaka_teleop.teleop:main',
            'compressedImage2dataset = kachaka_teleop.compressedImage2dataset:main',
            'image_to_video_node = kachaka_teleop.image2video:main',
            'get_pos_node = kachaka_teleop.getPosNode:main'     
        ],
    },
)
