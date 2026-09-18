from setuptools import find_packages, setup

package_name = 'voice_command'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name + '/resource',
            [
                'resource/.env',
                'resource/hello_rokey_8332_32.tflite',
            ]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eon',
    maintainer_email='seb000423@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'voice_command_node = voice_command.voice_command_client:main',
        ],
    },
)