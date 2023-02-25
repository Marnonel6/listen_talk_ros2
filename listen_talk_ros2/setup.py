from setuptools import setup

package_name = 'listen_talk_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'config/Hey-Willie_en_linux_v2_1_0.ppn', 'config/Dog-command_en_linux_v2_1_0.rhn', 'launch/listen_talk.launch.py', 'config/talk.mp3', 'config/German_shepherd_barking.mp3']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marno',
    maintainer_email='marthinusnel2023@u.northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listen = listen_talk_ros2.listen:main',
            'talk = listen_talk_ros2.talk:main'
        ],
    },
)
